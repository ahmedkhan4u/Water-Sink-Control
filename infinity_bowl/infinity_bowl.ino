#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <ezButton.h>
#include "RTClib.h"

/********** Global Variables Definitions *************/
char inputJson[250];
StaticJsonDocument<500> doc;

#define SEND_TEMPERATURE_DELAY 5
int sendTempertureCounter = 0;

//Schedule Preferences Definitions
Preferences preferences;
struct Preference {
  int scheduleId;    //Schedule Id is different from the id in DBHandler in Android, it's more of a Serial Number.
  int scheduleHour;
  int scheduleMin;
  int onOff;
  int mon;
  int tue;
  int wed;
  int thur;
  int fri;
  int sat;
  int sun;

} schedulesArray[10];

//Settings
int MINIMUM_TEMPERATURE = 1;  //Hardcoded
int MAXIMUM_TEMPERATURE = 30;
int FLUSH_TIME = 20;
int WAIT_TIME = 5;
int SENSOR_SETTLE_TIME = 2;

int FAIL_SAFE_TIMER_TIMEOUT = 60;

//Buttons Definition
ezButton flushButton(32);
ezButton fillButton(33);

//Valves Definition
boolean flushValveOn = false;
boolean fillValveOn = false;

int standyPin = 5;

int flushValvePin1 = 14;
int flushValvePin2 = 27;
int flushValvePWMPin = 15;

int fillValvePin1 = 26;
int fillValvePin2 = 25;
int fillValvePWMPin = 13;

boolean valveOperationFlag = false;   //This flag is used to turn on/off the valves only once, otherwise the valves input are repeated on every cycle
boolean removeBowlFlag = false;
//Sensor Definition
ezButton waterLevelSensor(4);

//Fail Safe LED/BUZZER Definition
int failSafeLedBuzzerPin = 12;

//Schedule Variables

/********************* RTC Definitions ****************************/
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
int previousMinute, currentMinute;

/********************* Sates Definitions ****************************/
enum States {
  FILL_UNTIL_SENSOR_TRIPPED, SENSOR_SETTLE_STATE, IDLE_STATE, FLUSH_FOR_SET_TIME, WAIT_STATE, FAIL_SAFE_TRIGGERED
};

//Variables
States state = FILL_UNTIL_SENSOR_TRIPPED;
int sensorSettleCounter = 0;
int failSafeTimerCounter = 0;
int failSafeTimeoutCounter = 0; //Used in the fill operation
int failSafeStateCounter = 0;  //Used in the fail safe triggered state
int flushCounter = 0;
int waitCounter = 0;
boolean waterLevelSensorTripped = false;

/********** Temperature Sensor Definitions ***********/
const int temperatureSensorPin = 34;
float voltageDividerR1 = 9850;
float BValue = 3950;
float R1 = 10000;
float base_T = 298.15;
float R2;
float measured_T;
//const int tempSensorNPNPin = 19;

/* Use for calculation in Temperature*/
float a;
float b;
float c;
float d;
float e = 2.718281828;
float tempSampleRead  = 0;
float tempLastSample = 0;
float tempSampleSum  = 0;
float tempSampleCount = 0;
float tempMean ;

/********* Function Prototypes ***********************/
void handleBLEAdvertising(void);
void parseJSONData(void);
void sendTemperatureOperation(void);
float getTemperatureReading(void);
void loadSchedulesFromPreferences(void);
void printSchedules(void);
void syncSchedule(String);
void syncSettings(String);
void loadSettingsFromPreferences(void);
void printSettings(void);
void buttonsOperation(void);
void turnFlushValveOn(void);
void turnFlushValveOff(void);
void turnFillValveOn(void);
void turnFillValveOff(void);
void stateMachine(void);
void resetState(void);
void checkIfOutsideTemperatureRange(void);
void printTime(void);
void checkForSchedule(void);
void print_wakeup_reason(void);

/****************======================== BLE Definitions =====================*****************/
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Deep Sleep---------------------------
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
int TIME_TO_SLEEP = 900;        /* Time ESP32 will go to sleep (in seconds) */
//#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex
RTC_DATA_ATTR int bootCount = 0;
int awakeTimeInSec = 30; // Time period to awake after deep sleep
int scheduleTimesArrSecs[10];
//--------------------------------------



class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      delay(250);
      Serial.println("Connected");
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      delay(250);
      Serial.println("Disconnected");
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      memset(inputJson, 0, sizeof inputJson);  //Clear this char array

      if (rxValue.length() > 0) {
        for (int i = 0; i < rxValue.length(); i++) {
          inputJson[i] = rxValue[i];
        }
        parseJSONData();
      }
    }
};
/****************************************=======================**********************************************/

/****** Timer Definition ******/
volatile int interruptCounter = 0;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

bool oneSecondFlag = false;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  oneSecondFlag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("Infinity Bowl");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE
                                          );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  //Timer Setup
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);

  //Buttons Setup
  flushButton.setDebounceTime(50);
  fillButton.setDebounceTime(50);

  //Valve Setup
  pinMode(standyPin, OUTPUT);
  //pinMode(tempSensorNPNPin, OUTPUT);
  pinMode(flushValvePin1, OUTPUT);
  pinMode(flushValvePin2, OUTPUT);
  pinMode(flushValvePWMPin, OUTPUT);

  pinMode(fillValvePin1, OUTPUT);
  pinMode(fillValvePin2, OUTPUT);
  pinMode(fillValvePWMPin, OUTPUT);

  //Initially Both valves will be off
  turnFlushValveOff();
  turnFillValveOff();

  //Fail Safe LED/BUZZER Setup
  pinMode(failSafeLedBuzzerPin, OUTPUT);
  digitalWrite(failSafeLedBuzzerPin, LOW);   //Initially Low

  // NPM Temperature Sensor /Pin
  //digitalWrite(tempSensorNPNPin, HIGH);
  Serial.println("NPN Sensor Check ON");

  //RTC Setup
  rtc.begin();
  //if (rtc.lostPower()) {
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //}

  //Initially set both variables to minute now
  DateTime now = rtc.now();
  previousMinute = now.minute();
  currentMinute = now.minute();

  //Preferences Setup
  preferences.begin("schedules", false);
  loadSchedulesFromPreferences();
  loadSettingsFromPreferences();

  if (deviceConnected == false) {
    state = IDLE_STATE;
    Serial.println("Device Connected => " + String(deviceConnected));
    delay(1000); //Take some time to open up the Serial Monitor
    //Increment boot number and print it every reboot
    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));
    //esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 1);
    print_wakeup_reason();
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);


    //digitalWrite(tempSensorNPNPin, LOW);
    Serial.println("Temperature Sensor NPN Off");
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
                   " Seconds");


    Serial.println("Going to sleep now");
    delay(1000);
    Serial.flush();
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  }

}

void loop() {
  Serial.println("Loop.........");


  if (oneSecondFlag) {
    stateMachine();
    oneSecondFlag = false;
  }

  handleBLEAdvertising();

  buttonsOperation();

  waterLevelSensor.loop();
  if (waterLevelSensor.isPressed()) waterLevelSensorTripped = true;  //Use in FILL_UNTIL_SENSOR_TRIPPED state.
}

void handleBLEAdvertising() {
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Start Advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}

void parseJSONData() {
  Serial.print("InputJson:");
  Serial.println(inputJson);

  String stringJSON = inputJson;
  DeserializationError error = deserializeJson(doc, inputJson);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }

  String command = doc["command"].as<String>();

  if (command == "sync_sch") {
    Serial.println("Sync Schedule");
    syncSchedule(stringJSON);

  } else if (command == "sync_sett") {
    Serial.println("Sync Settings");
    syncSettings(stringJSON);

  } else if (command == "flush_now") {
    Serial.println("Flush now");

    if (state == IDLE_STATE) {
      state = FLUSH_FOR_SET_TIME;

    } else {
      if (deviceConnected) {
        pTxCharacteristic->setValue("{\"response\":\"not_idle\"}");
        pTxCharacteristic->notify();
        delay(10);
      }
    }
  }
  else if (command == "remove_bowl") {
    Serial.println("Remove Bowl");
    removeBowlFlag = true;
    state = FLUSH_FOR_SET_TIME;
  }

  else if (command == "bowl_replaced") {
    Serial.println("Bowl Replaced");
    state = FILL_UNTIL_SENSOR_TRIPPED;
  }
}

void sendTemperatureOperation() {
  if (deviceConnected) {
    float temperatureC = getTemperatureReading();

    String dataString = "{\"response\":\"temp\", \"temp\":\"";
    dataString += temperatureC;
    dataString += "\"}";

    pTxCharacteristic->setValue(dataString.c_str());
    pTxCharacteristic->notify();
    delay(10);
  }
}

float getTemperatureReading() {
  for (int i = 0; i < 1000; i++) {
    tempSampleRead = analogRead(temperatureSensorPin);                                                 /* read analog value from sensor */
    tempSampleSum = tempSampleSum + tempSampleRead;                                             /* add all analog value for averaging later*/
    tempSampleCount = tempSampleCount + 1;                                                      /* keep counting the sample quantity*/
    tempLastSample = millis();                                                                  /* reset the time in order to repeat the loop again*/
  }

  tempMean = tempSampleSum / tempSampleCount;                                                 /* find the average analog value from those data*/
  R2 = (voltageDividerR1 * tempMean) / (4095 - tempMean);                                     /* convert the average analog value to resistance value*/
  a = 1 / base_T;                                                                                 /* use for calculation */
  b = log10(R1 / R2);                                                                         /* use for calculation */
  c = b / log10(e);                                                                           /* use for calculation */
  d = c / BValue ;                                                                            /* use for calculation */
  measured_T = 1 / (a - d);                                                                           /* the measured temperature value based on calculation (in Kelvin) */
  int tempInC = measured_T - 273.15;

  tempSampleSum = 0;                                                                          /* reset all the total analog value back to 0 for the next count */
  tempSampleCount = 0;

  return tempInC;
}

void syncSchedule(String scheduleJSON) {
  int scheduleId = doc["sc_id"].as<int>();
  String key = "schedule_";
  key += scheduleId;

  preferences.putString(key.c_str(), scheduleJSON);

  int onOff = doc["onOff"].as<int>();

  if (deviceConnected && onOff != -1) {
    String responseString = "{\"response\":\"s_sn\",\"sc_id\":"; //s_sn = Schedule Synced, sc_id = Scheudule Id (as Serial no not as in DBHandler)
    responseString += scheduleId;
    responseString += "}";
    pTxCharacteristic->setValue(responseString.c_str());
    pTxCharacteristic->notify();
    delay(10);
  }

  if (scheduleId == 10) {
    loadSchedulesFromPreferences();
    printSchedules();
  }
}

void loadSchedulesFromPreferences() {
  for (int i = 0; i < 10; i++) {
    String key = "schedule_";
    key += (i + 1);

    //Get Schedule JSON String for 10 schedules from Prefernces
    String scheduleJSON = preferences.getString(key.c_str(), "");

    scheduleJSON.trim();
    if (scheduleJSON != "") {
      DeserializationError error = deserializeJson(doc, scheduleJSON);
      JsonObject obj = doc.as<JsonObject>();

      // Test if parsing succeeds.
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());

        schedulesArray[i].scheduleId = -1;
        schedulesArray[i].scheduleHour = -1;
        schedulesArray[i].scheduleMin = -1;
        schedulesArray[i].onOff = -1;
        schedulesArray[i].mon = -1;
        schedulesArray[i].tue = -1;
        schedulesArray[i].wed = -1;
        schedulesArray[i].thur = -1;
        schedulesArray[i].fri = -1;
        schedulesArray[i].sat = -1;
        schedulesArray[i].sun = -1;

      } else {
        int scheduleId = doc["sc_id"].as<int>();
        int scheduleHour = doc["hour"].as<int>();
        int scheduleMin = doc["min"].as<int>();
        int onOff = doc["onOff"].as<int>();
        int mon = doc["mon"].as<int>();
        int tue = doc["tue"].as<int>();
        int wed = doc["wed"].as<int>();
        int thur = doc["thur"].as<int>();
        int fri = doc["fri"].as<int>();
        int sat = doc["sat"].as<int>();
        int sun = doc["sun"].as<int>();

        schedulesArray[i].scheduleId = scheduleId;
        schedulesArray[i].scheduleHour = scheduleHour;
        schedulesArray[i].scheduleMin = scheduleMin;
        schedulesArray[i].onOff = onOff;
        schedulesArray[i].mon = mon;
        schedulesArray[i].tue = tue;
        schedulesArray[i].wed = wed;
        schedulesArray[i].thur = thur;
        schedulesArray[i].fri = fri;
        schedulesArray[i].sat = sat;
        schedulesArray[i].sun = sun;
      }
    } else {
      schedulesArray[i].scheduleId = -1;
      schedulesArray[i].scheduleHour = -1;
      schedulesArray[i].scheduleMin = -1;
      schedulesArray[i].onOff = -1;
      schedulesArray[i].mon = -1;
      schedulesArray[i].tue = -1;
      schedulesArray[i].wed = -1;
      schedulesArray[i].thur = -1;
      schedulesArray[i].fri = -1;
      schedulesArray[i].sat = -1;
      schedulesArray[i].sun = -1;
    }
  }
}

void printSchedules() {
  Serial.println("");
  Serial.println("*************************************************************************************************************************");
  DateTime now = rtc.now();
  for (int i = 0; i < 10; i++) {
    Serial.print("Schedule ");
    Serial.print(i + 1);
    Serial.println(" :");

    Serial.print("Schedule Id: ");
    Serial.print(schedulesArray[i].scheduleId);
    Serial.print("  ");

    Serial.print("Hour: ");
    Serial.print(schedulesArray[i].scheduleHour);
    Serial.print("  ");

    Serial.print("Min: ");
    Serial.print(schedulesArray[i].scheduleMin);
    Serial.print("  ");
    // TODO
    int timeInSecs;
    int startTime = ((now.hour() * 60 * 60) + (now.minute() * 60));
    int endTime = ((schedulesArray[i].scheduleHour * 60 * 60) + (schedulesArray[i].scheduleMin * 60));
    if (schedulesArray[i].scheduleHour != -1) {
      timeInSecs = endTime - startTime;
      if (timeInSecs < 0) {
        //timeInSecs = endTime - startTime;
        //timeInSecs = (43200 - timeInSecs);
        //timeInSecs < 0 ? timeInSecs = 0 : timeInSecs;
        timeInSecs = 0;
      }
      //      else {
      //        timeInSecs = (43200 - timeInSecs);
      //        timeInSecs < 0 ? timeInSecs = 0 : timeInSecs;
      //      }
    }
    else {
      timeInSecs = 0;
    }
    scheduleTimesArrSecs[i] = timeInSecs;
    Serial.print(" Time In Secs ");
    Serial.print(timeInSecs);
    Serial.print("  ");

    Serial.print("On/Off: ");
    Serial.print(schedulesArray[i].onOff);
    Serial.print("  ");

    Serial.print("Monday: ");
    Serial.print(schedulesArray[i].mon);
    Serial.print("  ");

    Serial.print("Tuesday: ");
    Serial.print(schedulesArray[i].tue);
    Serial.print("  ");

    Serial.print("Wednesday: ");
    Serial.print(schedulesArray[i].wed);
    Serial.print("  ");

    Serial.print("Thursday: ");
    Serial.print(schedulesArray[i].thur);
    Serial.print("  ");

    Serial.print("Friday: ");
    Serial.print(schedulesArray[i].fri);
    Serial.print("  ");

    Serial.print("Saturday: ");
    Serial.print(schedulesArray[i].sat);
    Serial.print("  ");

    Serial.print("Sunday: ");
    Serial.println(schedulesArray[i].sun);

    Serial.println("-----------------------------------------------------------------------------------------------------------------------");
    Serial.println("");
  }

  // Finding Earliest Schedule Time To Assign TO Deep Sleep Time

  int eSchedule = 100000;

  for (int i = 0; i < 10; i++) {
    if (scheduleTimesArrSecs[i] == 0) continue;
    if (eSchedule > scheduleTimesArrSecs[i]) {
      eSchedule = scheduleTimesArrSecs[i];
    }
    Serial.print("Schedules Time In Seconds : ");
    Serial.println(scheduleTimesArrSecs[i]);
    Serial.println("");

  }
  if (eSchedule == 100000) {
    TIME_TO_SLEEP = 900;
  } else {
    TIME_TO_SLEEP = eSchedule > 900 ? TIME_TO_SLEEP = 900 : eSchedule;
    //TIME_TO_SLEEP = eSchedule;
  }
  Serial.println("Earliest Schedule Time : ");
  Serial.println(TIME_TO_SLEEP);
  Serial.println("*************************************************************************************************************************");
  Serial.println("");
}

void syncSettings(String stringJSON) {
  int max_temp = doc["max_temp"].as<int>();
  int flush_time = doc["flush_time"].as<int>();
  int wait_time = doc["wait_time"].as<int>();
  int sensor_settle_time = doc["sensor_settle_time"].as<int>();

  preferences.putInt("max_temp", max_temp);
  preferences.putInt("flush_time", flush_time);
  preferences.putInt("wait_time", wait_time);
  preferences.putInt("sen_set_tim", sensor_settle_time);

  long unixTimeLong = doc["unix_time"].as<int>();
  if (unixTimeLong != 0) {
    Serial.println("Adjusting RTC Time to unix time... ");
    Serial.println(unixTimeLong);
    rtc.adjust(DateTime(unixTimeLong));
  }

  if (deviceConnected) {
    String responseString = "{\"response\":\"set_sn\"}"; //set_sn = Settings Synced
    pTxCharacteristic->setValue(responseString.c_str());
    pTxCharacteristic->notify();
    delay(10);
  }

  loadSettingsFromPreferences();
  printSettings();
}

void loadSettingsFromPreferences() {
  MAXIMUM_TEMPERATURE = preferences.getInt("max_temp", 30); //Default = 30
  FLUSH_TIME = preferences.getInt("flush_time", 20); //Default = 20
  WAIT_TIME = preferences.getInt("wait_time", 5); //Default = 5
  SENSOR_SETTLE_TIME = preferences.getInt("sen_set_tim", 2); //Default = 2
}

void printSettings() {
  Serial.print("Maximum Temperature: ");
  Serial.println(MAXIMUM_TEMPERATURE);

  Serial.print("Flush Time: ");
  Serial.println(FLUSH_TIME);

  Serial.print("Wait Time: ");
  Serial.println(WAIT_TIME);

  Serial.print("Sensor Settle Time: ");
  Serial.println(SENSOR_SETTLE_TIME);
}

void buttonsOperation() {
  flushButton.loop();
  fillButton.loop();

  if (flushButton.isPressed()) {
    resetState();       //Override everything going on

    if (!flushValveOn) {
      turnFlushValveOn();
      flushValveOn = true;
    } else {
      turnFlushValveOff();
      flushValveOn = false;
    }
  }

  if (fillButton.isPressed()) {
    resetState();       //Override everything going on

    if (!fillValveOn) {
      turnFillValveOn();
      fillValveOn = true;

    } else {
      turnFillValveOff();
      fillValveOn = false;
    }
  }
}
void turnFlushValveOn() {
  //Flush Valve On
  Serial.println("Flush Valve On");
  digitalWrite (standyPin, HIGH);
  digitalWrite(flushValvePin1, LOW);
  digitalWrite(flushValvePin2, HIGH);
  digitalWrite(flushValvePWMPin, HIGH);

  delay (50);  //pulse on for 50 ms

  digitalWrite (flushValvePin1, LOW);
  digitalWrite (flushValvePin1, LOW);
  digitalWrite (standyPin, LOW);
  digitalWrite(flushValvePWMPin, LOW);
}

void turnFlushValveOff() {
  //Flush Valve Off
  Serial.println("Flush Valve Off");
  digitalWrite (standyPin, HIGH);
  digitalWrite(flushValvePin1, HIGH);
  digitalWrite(flushValvePin2, LOW);
  digitalWrite(flushValvePWMPin, HIGH);

  delay (50);  //pulse on for 50 ms

  digitalWrite (flushValvePin1, LOW);
  digitalWrite (flushValvePin1, LOW);
  digitalWrite (standyPin, LOW);
  digitalWrite(flushValvePWMPin, LOW);
}

void turnFillValveOn() {
  //Fill Valve On
  Serial.println("Fill Valve On");
  digitalWrite (standyPin, HIGH);
  digitalWrite(fillValvePin1, HIGH);
  digitalWrite(fillValvePin2, LOW);
  digitalWrite(fillValvePWMPin, HIGH);

  delay (50);  //pulse on for 50 ms

  digitalWrite (fillValvePin1, LOW);
  digitalWrite (fillValvePin1, LOW);
  digitalWrite (standyPin, LOW);
  digitalWrite(fillValvePWMPin, LOW);
}

void turnFillValveOff() {
  //Fill Valve Off
  Serial.println("Fill Valve Off");
  digitalWrite (standyPin, HIGH);
  digitalWrite(fillValvePin1, LOW);
  digitalWrite(fillValvePin2, HIGH);
  digitalWrite(fillValvePWMPin, HIGH);

  delay (50);  //pulse on for 50 ms

  digitalWrite (fillValvePin1, LOW);
  digitalWrite (fillValvePin1, LOW);
  digitalWrite (standyPin, LOW);
  digitalWrite(fillValvePWMPin, LOW);
}

void stateMachine() {
  Serial.print("State: ");

  switch (state) {
    case FILL_UNTIL_SENSOR_TRIPPED:
      Serial.println("FILL_UNTIL_SENSOR_TRIPPED");

      //Fill sink until Sensor Trips
      if (!valveOperationFlag) {   //Do once
        turnFlushValveOff();   //To be on safe side
        turnFillValveOn();

        valveOperationFlag = true;
      }


      if (waterLevelSensorTripped) {
        Serial.println("Water Level Sensor tripped!");
        state = SENSOR_SETTLE_STATE;
        failSafeTimeoutCounter = 0;
        waterLevelSensorTripped = false;

        valveOperationFlag = false;    //Resetting the flag is extremely important
      }

      //Secondary Fail Safe timer for fill operation
      if (failSafeTimeoutCounter >= FAIL_SAFE_TIMER_TIMEOUT) {
        turnFillValveOff(); //Turn off fill valve
        Serial.println("Fill Valve turned off");

        waterLevelSensorTripped = false;
        failSafeTimeoutCounter = 0;

        valveOperationFlag = false;    //Resetting the flag is extremely important

        state = FAIL_SAFE_TRIGGERED;
      }

      failSafeTimeoutCounter++;
      break;

    case SENSOR_SETTLE_STATE:
      Serial.println("SENSOR_SETTLE_STATE");

      if (sensorSettleCounter >= SENSOR_SETTLE_TIME) {
        turnFillValveOff();
        Serial.println("Fill Valve turned off");

        sensorSettleCounter = 0;
        state = IDLE_STATE; //Go to idle state
      }
      sensorSettleCounter++;
      break;

    case IDLE_STATE:
      Serial.println("IDLE_STATE");
      printTime();
      checkForSchedule();

      if (removeBowlFlag == true) {
        removeBowlFlag = false;
        Serial.print("Remove Bowl Flag : ");
        Serial.println(removeBowlFlag);
      }

      if (sendTempertureCounter >= SEND_TEMPERATURE_DELAY) {
        sendTemperatureOperation();
        sendTempertureCounter = 0;
      }

      checkIfOutsideTemperatureRange();

      sendTempertureCounter++;
      break;

    case FLUSH_FOR_SET_TIME:
      Serial.println("FLUSH_FOR_SET_TIME");
      //Turn on flush valve once

      if (!valveOperationFlag) {   //Do once
        turnFlushValveOn();
        turnFillValveOff();  //To be on safe side
        valveOperationFlag = true;
      }

      if (flushCounter >= FLUSH_TIME) {
        turnFlushValveOff();
        Serial.println("Flush Valve turned off");

        flushCounter = 0;

        valveOperationFlag = false;    //Resetting the flag is extremely important

        if (removeBowlFlag == true) {
          Serial.println("Remove Bowl Flush State");
          removeBowlFlag = true;
          state = IDLE_STATE;

          String responseString = "{\"response\":\"bowl_removed\"}"; //set_sn = Settings Synced
          pTxCharacteristic->setValue(responseString.c_str());
          pTxCharacteristic->notify();
          delay(10);

        } else {
          state = WAIT_STATE; //Go to wait state
        }


      }

      flushCounter++;
      break;

    case WAIT_STATE:
      Serial.println("WAIT_STATE");

      if (waitCounter >= WAIT_TIME) {
        waitCounter = 0;
        waterLevelSensorTripped = false;
        state = FILL_UNTIL_SENSOR_TRIPPED; //Go to fill until sensor trips state.
      }

      waitCounter++;
      break;

    case FAIL_SAFE_TRIGGERED:
      Serial.println("FAIL_SAFE_TRIGGERED");
      digitalWrite(failSafeLedBuzzerPin, !digitalRead(failSafeLedBuzzerPin));   //Invert the output

      if (failSafeStateCounter >= FAIL_SAFE_TIMER_TIMEOUT) {
        digitalWrite(failSafeLedBuzzerPin, LOW);   //Turn off the buzzer/led
        Serial.println("Going to idle.");
        failSafeStateCounter = 0;
        state = IDLE_STATE; //Go to idle state
      }

      failSafeStateCounter++;
      break;
  }
}

void resetState() {
  Serial.println("Reset State");
  state = IDLE_STATE;

  sensorSettleCounter = 0;
  failSafeTimerCounter = 0;
  failSafeTimeoutCounter = 0;
  failSafeStateCounter = 0;
  flushCounter = 0;
  waitCounter = 0;
  waterLevelSensorTripped = false;
}

void checkIfOutsideTemperatureRange() {
  float temperatureC = getTemperatureReading();

  Serial.print("Temp in C: ");
  Serial.println(temperatureC);

  if (temperatureC < MINIMUM_TEMPERATURE || temperatureC > MAXIMUM_TEMPERATURE) {
    Serial.println("Outside Temperature range, Flushing now...");

    if (deviceConnected) {
      pTxCharacteristic->setValue("{\"response\":\"out_tem_ran\"}");  //out_tem_ran = Outside Temperature range
      pTxCharacteristic->notify();
      delay(10);
    }

    valveOperationFlag = false;
    state = FLUSH_FOR_SET_TIME;
  }
}

void printTime() {
  DateTime now = rtc.now();

  char time_str[50];
  sprintf(time_str, "%u/%u/%u (%s) %02u:%02u:%02u ", now.year(), now.month(), now.day(), daysOfTheWeek[now.dayOfTheWeek()], now.hour(), now.minute(), now.second());
  Serial.println(time_str);
}

void checkForSchedule() {
  DateTime now = rtc.now();
  currentMinute = now.minute();
  Serial.println(String(previousMinute) + " === " + String(currentMinute));
  if (previousMinute != currentMinute) {          //a minute has passed, we will check for schedule every minute
    previousMinute = currentMinute;

    Serial.println("A minute has passed!");

    for (int h = 0; h < 10; h++) {
      int daysOfWeek[7] = {
        schedulesArray[h].sun, schedulesArray[h].mon, schedulesArray[h].tue, schedulesArray[h].wed, schedulesArray[h].thur,
        schedulesArray[h].fri, schedulesArray[h].sat
      }; //Sunday's flag is first because the rtc gives days of week in this order sun, mon, tue, wed, thur, fri, sat

      //The check for schedule
      /* == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == */

      int mScheduleHour = ((schedulesArray[h].scheduleHour) - 12);
      mScheduleHour = mScheduleHour < 0 ? schedulesArray[h].scheduleHour : mScheduleHour;

      Serial.println("---------------------------------------------------------------------------");
      Serial.println(now.hour());
      Serial.println(mScheduleHour);
      Serial.println(now.minute());
      Serial.println(schedulesArray[h].scheduleMin);
      Serial.println(daysOfWeek[now.dayOfTheWeek()]);
      Serial.println(schedulesArray[h].onOff);




      Serial.println("---------------------------------------------------------------------------");

      if (now.hour() == ((schedulesArray[h].scheduleHour) - 12) && now.minute() == schedulesArray[h].scheduleMin
          && daysOfWeek[now.dayOfTheWeek()] == 1 && schedulesArray[h].onOff == 1) {

        Serial.println("");
        Serial.print("Schedule ");
        Serial.print(schedulesArray[h].scheduleId);
        Serial.print(" matched!!!");
        Serial.println("");
        Serial.println("");
        state = FLUSH_FOR_SET_TIME;
        break;

      } else {
        Serial.print("Schedule ");
        Serial.print(schedulesArray[h].scheduleId);
        Serial.println(" not matched!");
      }
      /* == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == */
    }
  }
}

void print_wakeup_reason() {
  int mDelay = 0;
  previousMinute = previousMinute - 1;

  while (mDelay <= awakeTimeInSec) {
    loop();
    Serial.println("Wake State => " + String(mDelay));
    delay(1000);
    mDelay++;
  }
  printSchedules();
  mDelay = 0;

  while (deviceConnected) {
    loop();
    Serial.println("Device Connected => " + String(deviceConnected));
    delay(1000);
  }
  // TODO Deep Sleep
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
  delay(5000);
}
