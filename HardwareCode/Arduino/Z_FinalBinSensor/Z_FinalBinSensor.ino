#include <NewPing.h>                                  // Sonar Distance Sensor Library
#include <HX711.h>                                    // Weight Scale Liberary
#include <SparkFun_ADXL345.h>                         // SparkFun ADXL345 Library (Accelerometer for detecting Bin Lid open and close Events)
#include "PinChangeInterrupt.h"                       // For Display box Button Interrupts
#include "WiFiEsp.h"                                  // WiFi library for ESP8266 (AT Commands)
#include <Adafruit_Sensor.h>                          // Adafruit unified library for sensors needed for DHT (Temperature and Humidity Sensor)
#include "DHT.h"                                      // DHT (Temperature and Humidity Sensor) Library

#define SensorID  1                                   // ID of this Sensor Unit                                   
#define WIFI_SSID "fatima"                            // WIFI Network Name
#define WIFI_PASS "abcd1234"                          // WIFI Password
#define WIFI_LED  53                                  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define TRIGGER_PIN1  48                              // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN1     49                              // Arduino pin tied to echo pin on the ultrasonic sensor.
#define USPOWER_PIN1  47
#define TRIGGER_PIN2  33                              // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     32                              // Arduino pin tied to echo pin on the ultrasonic sensor.
#define USPOWER_PIN2  31
#define MAX_DISTANCE 500                              // Max distance the sonar sensor can measure as per datasheet (HC-SR04)
#define SCALE_DOUT_PIN 11                             // Green Wire in Prototype
#define SCALE_SCK_PIN 10                              // Blue Wire in Prototype
#define SCALE_CALIBRATION_FACTOR 10425.25             // Calilbaration Factor for the Weight Scale
#define LIDINTERRUPTPIN 2                             // The ADXL345 (Accelerometer for detecting Bin Lid open and close Events) will trigger interrupt on Arduino Mega digital pin 2
#define DHTPIN 8                                      // DHT22 (Temperature and Humidity Sensor) is connected to Arduino Mega digital pin 8
#define DHTTYPE DHT22                                 // DHT 22  (AM2302), AM2321, The type of DHT sensor being used
#define DEBUG true                                    // Global Flag to disable Serial printing in deployment mode
String SERVER = "178.159.10.106";                     // InfluxDB server to post data too
int PORT = 8086;



// Initialize the Wifi client object
WiFiEspClient client;                                 // Wifi radio client object
int status = WL_IDLE_STATUS;                          // the Wifi radio's status
char server[] = "178.159.10.106";                     // The InfluxDB server Address to send data to

//Temperature Humidity Sensor
DHT dht(DHTPIN, DHTTYPE);                             // DHT(Temperature and Humidity Sensor) Sensor pin and DHT sensor type (DHT22 in this case)

//Distance Sensor for fill level sensing
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE); // Declaration for HC-SR04 sonar distance sensor 1 using NewPing library
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE); // Declaration for HC-SR04 sonar distance sensor 2 using NewPing library

// Scale Settings for weight sensing
HX711 scale(SCALE_DOUT_PIN, SCALE_SCK_PIN);            // Declaration for weight sensor using HX711 library

// Gas Sensors
int GasSensorPins[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8}; // Pins for MQ gas Sensors in the samse order as declaration MQ2,MQ3,MQ4,MQ5,MQ6,MQ7,MQ8,MQ9,MQ135

//Accelerometer for Lid open and close event detection
ADXL345 adxl = ADXL345();                             // Declaration ADXL345 (Accelerometer for detecting Bin Lid open and close Events) using I2C COMMUNICATION, uses pin 20 (SDA) and 21 (SCL) on Arduino mega
volatile boolean lidActivity = false;                 // flag to detect if there is a change in the lid state
int lidEventLED = 53;
int LidStatusNow = 1;                                 // Current status of the bin lid (0 Open, 1 Closed)
int LidPreviousStatus = 1;                            // Previous status of the bin lid (0 Open, 1 Closed)
unsigned long lastLidInterrupt;                       // Time in mili secs when last lid interrupt occured
unsigned long delayLisInterrupt = 500;                // Ideal delay in mili secs between interrupts (tested on prototype is 350 ms)
boolean triggerImage = false;                         // Triggered when lid is opened and then closed

//Display box
int displayLEDS[] = {22, 23, 24, 25, 26};             // Led confirmation for each button pressed on the Display box (Each pin corresponds to the button pin in the same order)
int displayButtons[] = {A9, A10, 50, 52, 51};         // Button pins (using PinChangeInterrupt library) for confirmation for item dropped in he bin (out of 5 possible items) Each pin corresponds to the led pin in the same order.
volatile boolean buttonEvnts[] = {false, false, false, false, false};

//Global Variables
float GasSensorValues[9]; //MQ2,MQ3,MQ4,MQ5,MQ6,MQ7,MQ8,MQ9,MQ135
float  Distance, Distance1, Distance2;
float Weight, Temperature, Humidity;


void setup() {
  Serial.begin(9600);
  Serial3.begin(115200);
  pinMode(USPOWER_PIN1, OUTPUT);
  digitalWrite(USPOWER_PIN1, HIGH);
  pinMode(USPOWER_PIN2, OUTPUT);
  digitalWrite(USPOWER_PIN2, HIGH);

  delay(100);
  for (int i = 0; i < 9; i++) {
    pinMode(GasSensorPins[i], INPUT);
    GasSensorValues[i] = 0;
  }

  pinMode(TRIGGER_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIGGER_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);

  for (int i = 0; i < 5; i++) {
    pinMode(displayLEDS[i], OUTPUT);
    pinMode(displayButtons[i], INPUT_PULLUP);
  }

  attachPCINT(digitalPinToPCINT(displayButtons[0]), button1Isr, FALLING);
  attachPCINT(digitalPinToPCINT(displayButtons[1]), button2Isr, FALLING);
  attachPCINT(digitalPinToPCINT(displayButtons[2]), button3Isr, FALLING);
  attachPCINT(digitalPinToPCINT(displayButtons[3]), button4Isr, FALLING);
  attachPCINT(digitalPinToPCINT(displayButtons[4]), button5Isr, FALLING);

  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  delay(500);
  dht.begin();
  delay(500);
  initADXL345();

  initWifi();
  Distance = 0;
  Weight = 0;

}

void loop() {
  readDHT22Readiings();
  checkAllInterrupts();
  readAvgScaleReadings(5);
  checkAllInterrupts();
  getFillDistance();
  checkAllInterrupts();
  readAvgGasSensor(10);
  checkAllInterrupts();
  if (triggerImage) {
    Serial.println("Take Image");
    triggerImage = false;
  }
  String readvalues = "SensorData,SensorID=" + String(SensorID) + " Temperature=" + String(Temperature) + ",Humidity=" + String(Humidity);
  readvalues = readvalues + ",MQ2=" + String(GasSensorValues[0]) + ",MQ3=" + String(GasSensorValues[1]) + ",MQ4=" + String(GasSensorValues[2]);
  readvalues = readvalues + ",MQ5=" + String(GasSensorValues[3]) + ",MQ6=" + String(GasSensorValues[4]) + ",MQ7=" + String(GasSensorValues[5]);
  readvalues = readvalues + ",MQ8=" + String(GasSensorValues[6]) + ",MQ9=" + String(GasSensorValues[7]) + ",MQ135=" + String(GasSensorValues[8]);
  readvalues = readvalues + ",DistanceLeft=" + String(Distance1) +",DistanceRight=" + String(Distance2) + ",Weight=" + String(Weight);
  readvalues = readvalues  + ",LidStatus=" + String(LidStatusNow) + ",ImageTriggered=" + String(triggerImage);
  Serial.println(readvalues);
  sendData(readvalues);
  delay(4000); // Print value every 4 sec.
}

void initWifi() {
  WiFi.init(&Serial3);
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }
  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_SSID);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  // you're connected now, so print out the data
  if (status == WL_CONNECTED) {
    Serial.println("You're connected to the network");
  }
  if (DEBUG) {
    printWifiStatus();
  }
}

boolean sendData(String postStr) {
  if (status != WL_CONNECTED) {
    initWifi();
  }else{
    digitalWrite(WIFI_LED, HIGH);
  }
  boolean sendOk = false;
  Serial.println("Starting connection to server...");
  // if you get a connection, report back via serial
  if (client.connect(server, PORT)) {
    Serial.println(postStr);
    client.print("POST /write?db=binsensor HTTP/1.1\n");
    client.print("Host: " + SERVER + "\n");
    client.print("Connection: close\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(postStr.length());
    client.print("\n\n");
    client.print(postStr);
    delay(1000);
    while (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.println(line);
      if (line.indexOf("204 No Content") > 0) {
        Serial.println("Post to Server successfull");
        sendOk = true;
        doblink(WIFI_LED, 2,500);
      }
    }

    //  // if the server's disconnected, stop the client
    if (!client.connected()) {
      Serial.println();
      Serial.println("Disconnecting from server...");
      client.stop();
    }
  }
  return sendOk;
}

void readDHT22Readiings() {
  Humidity = dht.readHumidity();
  Temperature = dht.readTemperature() - 4;
  // Check if any reads failed and exit early (to try again).
  if (isnan(Humidity) || isnan(Temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    Humidity = 0.0;
    Temperature = 0.0;
  }

}

void readAvgScaleReadings(int nSamples) {
  scale.set_scale(SCALE_CALIBRATION_FACTOR);
  delay(50);
  float currentWeight = 0.0;
  for (int i = 0; i < nSamples; i++) {
    currentWeight = currentWeight + scale.get_units(1) * 0.453592;
    if (currentWeight < 0) {
      currentWeight = 0;
    }
    delay(25);
  }
  Weight = currentWeight / nSamples;
}

void readAvgGasSensor(int nSamples) {
  for (int i = 0; i < 9; i++) {
    GasSensorValues[i] = 0;
  }
  for (int j = 1; j <= nSamples; j++) {
    for (int i = 0; i < 9; i++) {
      GasSensorValues[i] = GasSensorValues[i] + (float) analogRead(GasSensorPins[i]);
    }
  }
  for (int i = 0; i < 9; i++) {
    GasSensorValues[i] = (float) GasSensorValues[i] / nSamples;
  }
}

void getFillDistance() {
  Distance1 = getSonarDistance(10, ECHO_PIN1, USPOWER_PIN1, sonar1);
  Distance2 = getSonarDistance(10, ECHO_PIN2, USPOWER_PIN2, sonar2);
  long diff = abs(Distance2 - Distance1);
  if (diff > 30) {
    Distance1 = getSonarDistance(10, ECHO_PIN1, USPOWER_PIN1, sonar1);
    Distance2 = getSonarDistance(10, ECHO_PIN2, USPOWER_PIN2, sonar2);
  }
  Distance = (Distance1 + Distance2) / 2;
}

long getSonarDistance(int maxiter, int epin, int powerpin, NewPing snr) {
  long distance = snr.ping_cm();
  delay(30);                     // Wait 30ms between pings (about 30 pings/sec). 29ms should be the shortest delay between pings.
  int  timeoutcntr = 0;
  while ((distance == 0) && (digitalRead(epin) == HIGH) && (timeoutcntr < maxiter)) {
    digitalWrite(powerpin, LOW);
    delay(50);                    // 50 mS is the minimum Off time to get clean restart
    digitalWrite(powerpin, HIGH); // Adjust this value if your sensors don't read after reset
    delay(180);                   // Some sensors throw out a very short false echo after timeout
    snr.ping_cm();                // 5cm-10cm value.  If your sensors do not throw out
    // this false echo, you can get rid of this delay and ping after power HIGH
    // Conversely, if you still get a false echo, you may need to increase delay
    timeoutcntr = timeoutcntr + 1;
    delay(50);                    // 50 mS is the minimum Off time to get clean restart
  }
  return distance;
}

void initADXL345() {
  // Power on the ADXL345
  adxl.powerOn();
  // Give the range settings
  // Accepted values are 2g, 4g, 8g or 16g
  // Higher Values = Wider Measurement Range
  // Lower Values = Greater Sensitivity
  adxl.setRangeSetting(2);
  adxl.setActivityXYZ(0, 1, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(10);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
  // Setting interupts to take place on INT1 pin
  // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
  // This library may have a problem using INT2 pin. Default to INT1 pin.
  adxl.setImportantInterruptMapping(2, 2, 2, 1, 2);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);"
  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  adxl.InactivityINT(0);
  adxl.ActivityINT(1);
  adxl.FreeFallINT(0);
  adxl.doubleTapINT(0);
  adxl.singleTapINT(0);
  attachInterrupt(digitalPinToInterrupt(LIDINTERRUPTPIN), lidEvent, RISING);   // Attach Interrupt
}

void checkAllInterrupts() {
  checkBinLidStatus();
  checkButtonInputs();
}

void checkBinLidStatus() {
  int x, y, z;
  adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z
  byte interrupts = adxl.getInterruptSource();
  if (lidActivity) {
    if (adxl.triggered(interrupts, ADXL345_ACTIVITY)) {
      digitalWrite(lidEventLED, HIGH);
      delay(500);
      digitalWrite(lidEventLED, LOW);

      if (y < 0) { //Lid is closed 0 Open, 1 Closed
        LidStatusNow = 1;
      } else if (y > 150) {
        LidStatusNow = 0;
      }
      Serial.println(String(x) + "," + String(y) + "," + String(z) + " Lid Now " + String(LidStatusNow) + " Lid Previously " + String(LidPreviousStatus));
      if ((LidPreviousStatus == 0) && (LidStatusNow == 1)) {
        triggerImage = true;
      }
      LidPreviousStatus = LidStatusNow;
    }
    lidActivity = false;
  }
}
void lidEvent() {
  if (millis() - lastLidInterrupt > delayLisInterrupt)
  {
    lastLidInterrupt = millis();
    lidActivity = true;
  }

}

void button1Isr() {
  buttonEvnts[0] = true;
}
void button2Isr() {
  buttonEvnts[1] = true;
}
void button3Isr() {
  buttonEvnts[2] = true;
}
void button4Isr() {
  buttonEvnts[3] = true;
}
void button5Isr() {
  buttonEvnts[4] = true;
}

void checkButtonInputs() {
  for (int i = 0; i < 5; i++) {
    if (buttonEvnts[i]) {
      buttonEvnts[i] = false;
      Serial.println("Blinking led " + String(i));
      doblink(displayLEDS[i], 2, 500);
    }
  }
}

void doblink(int lpin, int ntimes, int msdelay) {
  for (int i = 0; i < ntimes; i++) {
    digitalWrite(lpin, HIGH);
    delay(msdelay);
    digitalWrite(lpin, LOW);
    delay(msdelay);
  }
}


void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
