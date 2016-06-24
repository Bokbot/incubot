#include <OneWire.h>
#include <PID_v1.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;

// constants won't change. Used here to set a pin number :
const int heaterPin =  13;      // the number of the LED pin

// Variables will change :
int heaterState = LOW;  
int heaterOn = 0;
unsigned long MySetPoint = 10050;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long nudge = 0;
unsigned long yank = 0;

// constants won't change :
const long interval = 12249;  

float tempC;

void setup(void)
{
  // start serial port
  Serial.begin(115200);
  //Serial.println("Dallas Temperature IC Control Library Demo");

  // locate devices on the bus
  //Serial.print("Locating devices...");
  sensors.begin();
  //Serial.print("Found ");
  //Serial.print(sensors.getDeviceCount(), DEC);
  //Serial.println(" devices.");

  // report parasite power requirements
  //Serial.print("Parasite power is: "); 
  //if (sensors.isParasitePowerMode()) Serial.println("ON");

  Setpoint = MySetPoint;
  //else Serial.println("OFF");
   myPID.SetOutputLimits(0, 25000);
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
  Serial.print("nudge");
  Serial.print(nudge);
  Serial.print(" countdown  ");
  Serial.print( interval + nudge - (millis() - previousMillis));
  Serial.print(" Temp F: ");
  Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
}

void loop(void)
{
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  //Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  //Serial.println("DONE");
  // It responds almost immediately. Let's print out the data
  printTemperature(insideThermometer);// Use a simple function to print out the data
  int tempF = 100 * DallasTemperature::toFahrenheit(tempC);
  if(tempF > 10076){
    heaterOn = 0;
    digitalWrite(heaterPin, HIGH);
  }
  if( tempF > 10){
    if( tempF < 9996){
      heaterOn = 1;
      //digitalWrite(heaterPin, LOW);
    }
  }
    unsigned long currentMillis = millis();
  if (heaterOn){
  if (currentMillis - previousMillis >= interval + nudge) {
    // save the last time you blinked the LED
    //previousMillis = currentMillis;
    nudge = 0;

    Input = tempF;
    myPID.Compute();
    // if the LED is off turn it on and vice-versa:
    if (heaterState) {
      previousMillis = currentMillis;
      heaterState = LOW;
      nudge = 0;
      if(tempF < MySetPoint){
      //nudge = 2000 + 400 * (MySetPoint - tempF);
        nudge = Output;
      }
    } else {
      previousMillis = currentMillis; //add 5 seconds of off time
      heaterState = HIGH;
      nudge = 25000;
      if(tempF > MySetPoint){
        //nudge = 25000 + 400 * (tempF - MySetPoint);
        nudge = 45000 - Output;
      }
      if(tempF < MySetPoint){
        nudge = 35000 - Output;
      }
    }

    // set the LED with the ledState of the variable:
    digitalWrite(heaterPin, heaterState);
  }
  }
  delay(750);
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
