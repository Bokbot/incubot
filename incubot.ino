// Enable debug prints
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#include <OneWire.h>
#include <PID_v1.h>
#include <DallasTemperature.h>

#include <SPI.h>
#include <MySensors.h>
#include <DHT.h>
#include <avr/wdt.h>

// Define Pins here
// Data wire is plugged into port 2 on the Arduino
#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define ONE_WIRE_BUS 3
#define HUMIDITY_SENSOR_DIGITAL_PIN 2
// constants won't change. Used here to set a pin number :
#define HEATER_PIN 5

#define CHILD_ID_HUM   0
#define CHILD_ID_TEMP  1
#define CHILD_ID_TEMP2 2
#define CHILD_ID_TEMP3 3
#define CHILD_ID_HVAC  53

// You might need to tune these for your setup
unsigned long SLEEP_TIME = 1000; // Sleep time between reads (in milliseconds)
const int sensorwatchdogLimit = 30;
const unsigned long sensorInterval = 30000; // 30,000 ms = 30 seconds
unsigned long WindowSize = 20000;
float outpercent = 0;

DHT dht;
float lastTemp;
float lastTempF;
float lastPercent;
float lastHum;
boolean metric = true; 
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgTemp2(CHILD_ID_TEMP2, V_TEMP);
MyMessage msgTemp3(CHILD_ID_TEMP3, V_TEMP);
// hvac for incubot needs work still
//MyMessage msgHVAC(CHILD_ID_HVAC, V_TEMP);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;

// Variables will change :
int heaterState = LOW;
int heaterOn = 0;

double MyMaxPoint = 101.00;
double MySetPoint = 100.30;
double MyMinPoint = 99.65;

//Define Variables we'll be connecting to
double Setpoint, Input;
double Output = 1000;

//Specify the links and initial tuning parameters
double Kp=100, Ki=500, Kd=200;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long windowStartTime;

unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long nudge = 0;
unsigned long yank = 0;


float tempC;

unsigned long sensorthrottle = 1000;
int sensorwatchdog = 0;

bool checkThrottle(unsigned long throttle, int dog, int watchdogLimit){

  if( millis() > throttle ) {
    // return one or 'ok'
    return 1;
  }
  else if( dog > watchdogLimit ){
    // return one or 'ok'
    return 1;
  }
  else{
    return 0;
  }
}

void turnHeatOn(){
  digitalWrite(HEATER_PIN, LOW);
  delay(300);
}

void turnHeatOff(){
  digitalWrite(HEATER_PIN, HIGH);
  delay(300);
}

void setup(void)
{
  wdt_enable(WDTO_8S);
  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN);
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, HIGH);

  metric = getConfig().isMetric;
  // start serial port
  Serial.begin(115200);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // locate devices on the bus
  //Serial.print("Locating devices...");
  sensors.begin();
  //Serial.print("Found ");
  //Serial.print(sensors.getDeviceCount(), DEC);
  //Serial.println(" devices.");

  // report parasite power requirements
  //Serial.print("Parasite power is: "); 
  //if (sensors.isParasitePowerMode()) Serial.println("ON");

  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = MySetPoint;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  // assign address manually.  the addresses below will beed to be changed
  // to valid device addresses on your bus.  device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  //insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };

  // Method 1:
  // search for devices on the bus and assign based on an index.  ideally,
  // you would do this to initially discover addresses on the bus and then 
  // use those addresses and manually assign them (see above) once you know 
  // the devices on your bus (and assuming they don't change).
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  
  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices, 
  // or you have already retrieved all of them.  It might be a good idea to 
  // check the CRC to make sure you didn't get garbage.  The order is 
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to insideThermometer
  //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");

  // show the addresses we found on the bus
  //Serial.print("Device 0 Address: ");
  //printAddress(insideThermometer);
  //Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 12);
 
  //Serial.print("Device 0 Resolution: ");
  //Serial.print(sensors.getResolution(insideThermometer), DEC); 
  //Serial.println();
}

void presentation()
{
  // Send the Sketch Version Information to the Gateway
  sendSketchInfo("Incubot", "0.1");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_TEMP2, S_TEMP);
  present(CHILD_ID_TEMP3, S_TEMP);
  present(CHILD_ID_HVAC, S_HVAC);
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  tempC = sensors.getTempC(deviceAddress);
  //Serial.print("Temp C: ");
  //Serial.print(tempC);
  //Serial.print(" Temp F: ");
  Serial.print("hState ");
  Serial.print(heaterState);
  Serial.print(" hOn ");
  Serial.print(heaterOn);
  Serial.print(" Set ");
  Serial.print(MySetPoint);
  Serial.print(" Min ");
  Serial.print(MyMinPoint);
  Serial.print(" In ");
  Serial.print(Input);
  Serial.print(" Out ");
  Serial.print(Output);
  Serial.print(" / ");
  Serial.print(WindowSize);
  Serial.print(" = ");
  Serial.print(outpercent);
  Serial.print("% ");
  Serial.print(" nudge ");
  Serial.print(nudge);
  Serial.print(" millis  ");
  Serial.print( millis() );
  Serial.print(" sensorthrottle  ");
  Serial.print( sensorthrottle );
  //Serial.print(" countdown  ");
  //Serial.print( millis() - windowStartTime - Output );
  Serial.print(" watchdog  ");
  Serial.print( sensorwatchdog );
  Serial.print(" Temp F: ");
  Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
}

void loop(void)
{

  wdt_reset();
  unsigned long currentMillis = millis();
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  //Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  //Serial.println("DONE");
  // It responds almost immediately. Let's print out the data
  printTemperature(insideThermometer);// Use a simple function to print out the data
  float tempF = DallasTemperature::toFahrenheit(tempC);
  Input = tempF;
  myPID.Compute();
  outpercent = 100 * (Output / WindowSize);
  if(Input > MyMaxPoint){
    heaterOn = 0;
    turnHeatOff();
  }
  // ignore negative
  if( Input > 0){
    if( Input < MySetPoint){
      heaterOn = 1;
      //digitalWrite(HEATER_PIN, LOW);
      //turnHeatOn();
    }
    if( Input < MyMinPoint){
      heaterOn = 0;
      heaterState = 1;
      turnHeatOn();
    }
  }

  if ((currentMillis - windowStartTime) > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

  if (heaterOn){
     /************************************************
     * turn the output pin on/off based on pid output
     ************************************************/
    if (Output < (currentMillis - windowStartTime)) {
      turnHeatOff();
      heaterState = 0;
    }
    else {
      turnHeatOn();
      heaterState = 1;
    }
  }
  //delay(dht.getMinimumSamplingPeriod());
 
  //if(checkThrottle( sensorthrottle, sensorwatchdog, sensorwatchdogLimit) || sensorwatchdog > 45){
  if(checkThrottle( sensorthrottle, sensorwatchdog, sensorwatchdogLimit)){
    delay(200); //sleep a bit

    sensorthrottle = (currentMillis + sensorInterval); 
    sensorwatchdog = 0;
      // Fetch temperatures from DHT sensor
      float temperature = dht.getTemperature();
      if (isnan(temperature)) {
        //  Serial.println("Failed reading temperature from DHT");
      } else if (temperature != lastTemp) {
        lastTemp = temperature;
        if (!metric) {
          //temperature = dht.toFahrenheit(temperature);
        }
        send(msgTemp.set(temperature, 1));
        #ifdef MY_DEBUG
        Serial.print("T: ");
        Serial.println(temperature);
        #endif
      }

      if (isnan(tempF)) {
        //  Serial.println("Failed reading tempF from DHT");
      } else if (tempF != lastTempF) {
        lastTempF = tempF;
        if (!metric) {
          //tempF = dht.toFahrenheit(tempF);
        }
        delay(200); //sleep a bit
        send(msgTemp2.set(tempF, 1));
        #ifdef MY_DEBUG
        Serial.print(" T2: ");
        Serial.println(tempF);
        #endif
      }
      

      if (isnan(outpercent)) {
        //  Serial.println("Failed reading outpercent from DHT");
      } else if (outpercent != lastPercent) {
        lastPercent = outpercent;
        if (!metric) {
//          outpercent = dht.toFahrenheit(outpercent);
        }
        delay(200); //sleep a bit
        send(msgTemp3.set(outpercent, 1));
        #ifdef MY_DEBUG
        Serial.print(" T3: ");
        Serial.println(outpercent);
        #endif
      }
      
      // Fetch humidity from DHT sensor
      float humidity = dht.getHumidity();
      if (isnan(humidity)) {
          //Serial.println("Failed reading humidity from DHT");
      } else if (humidity != lastHum) {
          lastHum = humidity;
          delay(200); //sleep a bit
          send(msgHum.set(humidity, 1));
          #ifdef MY_DEBUG
          Serial.print(" H: ");
          Serial.println(humidity);
          #endif
      }
  }
  
  sensorwatchdog++;
  // sleep may not be a good idea with the relay involved
  //sleep(SLEEP_TIME); //sleep a bit
  delay(SLEEP_TIME); //sleep a bit
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
