/* Chip McClelland - Cellular Data Logger
BSD license, Please keep my name and credits in any redistribution

Requirements: 
  - Account on Ubidots.  http://www.ubidots.com
  - Adafruit CC3000 Breakout Board - see below for link
  - Adafruit Ultimate GPS: https://www.adafruit.com/product/746
  - Teensy 3.1
Credits:
  - Ubidots Demo Code from: Mateo Velez for Ubidots,Inc. Modified 15 Nov 2014
Configuration:
  - You will need to add your SSID and password information for WiFi (see xxx below)
  - You will also need to add keys for your account and variable on Ubidots (see xxxx below)
    Hardware Connections: (Teensy 3.1)
    Adafruit CC3000 Breakout   Function
    VCC        +3.3V           External Power - not from Teensy
    GND        GND             GND
    2          INT             Interrupt (Must be an interrupt pin - Mico has 5)
    5          EN              WiFi Enable (Can be any ping)
    10         CS              SPI Chip Select
    11         MOSI            SPI MOSI
    12         MISO            SPI MISO
    13         SCK             SPI Clock
    Adafruit Ultimate GPs     
    VCC        +3.3V           External Power - not from Teensy
    GND        GND
    7          Enable          Turn on and off module
    8          Fix             Do we have a fix?
    0          TX->            Hardware UART Rx
    1          RX<-            Hardware UART Tx
    Sparkfun MMA8452 Breakout
    VCC        +3.3V          External Power - not from Teensy
    GND         GND
    18          SDA0          i2c data - should have 4.7k pullup 
    19          SCL0          i2c clock - should have 4.7k pullup    
    3           I2            To wake up on tap - should have 4.7k pullup
    4           I1            Not used by wired for future use - should have 4.7k pullup
    Sensitivity 10k Trim Pots
    14          A0            Delay Time
    15          A1            Sensitivity Time
    Indicator Leds (all three have switched ground) 
    16          LED1          Tap - Red
    17          LED2          TBD - Yellow
    VCC         LED3          Power
  /******************** Library Credits ******************************* 
  This is an example for the Adafruit CC3000 Wifi Breakout & Shield

  Designed specifically to work with the Adafruit WiFi products:
  ----> https://www.adafruit.com/products/1469

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution

 Improved Wire Library for Teensy 3.1 Used below (Wire.h works as well)
 Credit - Brian - Nox77 - https://github.com/nox771/i2c_t3
 Thanks Brian!
 
 
****************************************************************/
// Included Libraries
#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"
#include <stdlib.h>
#include <GPS.h>      // Code and Library from: https://github.com/rvnash/ultimate_gps_teensy3
#include <WProgram.h>
#include <i2c_t3.h>

// Set up the GPS using Hardware Serial1 - Pins 0 and 1
HardwareSerial &gpsSerial = Serial1;
GPS gps(&gpsSerial,true);

// Set up the WiFi Module
#define CC3000_IRQ   2  // MUST be an interrupt pin!
#define CC3000_VBAT  5  // These can be any two pins
#define CC3000_CS    10 // Use hardware SPI for the remaining pins
Adafruit_CC3000 cc3000 = Adafruit_CC3000(CC3000_CS, CC3000_IRQ, CC3000_VBAT, SPI_CLOCK_DIVIDER); // you can change this clock speed

#define MOBILE 1
#if MOBILE
#define WLAN_SSID       "xxxxxxx"           // Personal LTE Hotspot
#define WLAN_PASS       "xxxxxxx"
#else
#define WLAN_SSID       "xxxxxxx"  // cannot be longer than 32 characters!
#define WLAN_PASS       "xxxxxxx"
#endif
#define WLAN_SECURITY   WLAN_SEC_WPA2 // Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 

// Set up the Accelerometer
const int MMA8452_ADDRESS = 0x1D; // The SparkFun MMA8452 breakout board defaults to 1 for an address of 0x1D
const byte SCALE = 2;  // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
const byte dataRate = 3;  // output data rate - 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
const int int1Pin = 4;         // Not used now but wired for future use
const int int2Pin = 3;         // This is the interrupt pin that registers taps

// Set up the TrimPots and Indicator Leds
const int TrimPot = A0;        // Potentiometer used to adjust sensitivity
const int TrimPot1 = A1;       // Potentiometer used to adjust delay
const int ledPin = 16;         // Tap Indicator LED
const int ledPin1 = 17;         // TBD


// Set up the Ubidots Account and Variables
#define WEBSITE      "things.ubidots.com"
String token = "xxxxxxxxxxxxx";  // For your account
String idvariable = "xxxxxxxxxxxxx";  // For your variable

// Global Variables
uint32_t ip;
int value = 0;
unsigned int ReportingInterval = 10000;  // How often do you want to send to Ubidots (in millis)
unsigned long LastReport = 0;            // Keep track of when we last sent data
int TimeOut = 4000;                      // How long will we wait for a command to complete
char c;                                  // Used to relay input from the GPS serial feed
String Location = "";                    // Will build the Location string here
int InputValue = 0;            // Raw sensitivity input
byte Sensitivity = 0x00;       // Hex variable for sensitivity
static byte source;
int ledState = LOW;            // variable used to store the last LED status, to toggle the light
unsigned long LastKnock = 0;   // Used for debouncing
int Debounce = 1000;           // Time in ms between knocks

 
void setup() 
{
  // Initialize Serial ports
  Serial.begin(19200);  // Start serial terminal
  delay(2000);
  Serial.println();
  Serial.println(F("---------------------------"));
  Serial.println(F("Connected Location Logger  "));
  Serial.println(F("---------------------------"));
  gps.startSerial(9600);  // Start serial1 for GPS
  gps.setSentencesToReceive(OUTPUT_RMC_GGA);  // Sets the format for the GPS sentences
  Wire.begin(); // Start serial on i2c

  // Initialize the Accelerometer
  pinMode(int2Pin, INPUT);     // Set up the Accelerometer interrupt pins, they're set as active high, push-pull
  digitalWrite(int2Pin, HIGH);
  pinMode(int1Pin, INPUT);   
  digitalWrite(int2Pin, HIGH);
  Serial.print("Initializing the accelerometer: ");
  byte c = readRegister(MMA8452_ADDRESS, 0x0D);            // Read WHO_AM_I register to test communications
  if (c == 0x2A) {                  // WHO_AM_I should always be 0x2A
    initMMA8452(SCALE, dataRate);   // init the accelerometer if communication is OK
    Serial.println("Succeeded");
  }
  else {                          // Problem with communucations
    Serial.print("Failed at address: 0x");
    Serial.println(c, HEX);
    while(1);                   // Loop forever if communication doesn't happen
  }
  // Initialise the Wi-Fi module
  Serial.print("Initializing CC3000 Wi-Fi: ");
  if (!cc3000.begin())
  {
    Serial.println(F("Failed"));
    while(1);
  }
  Serial.println("Succeeded");
  GetConnected();             // Test connection to Internet
  Serial.print(WEBSITE); 
  Serial.print(F(" -> "));   // Try looking up the website's IP address
  while (ip == 0) {
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(1000);  // give it a little more time
  }
  cc3000.printIPdotsRev(ip);  // Success! Print restuls
  while (! displayConnectionDetails()) {  // Display the IP address DNS, Gateway, etc.  
    delay(1000);
  }
 // GetDisconnected();  // If you want to save batteries - let the logger disconnect (two comments below)
 Serial.println("Location Logger is on-line any movement will be logged");
}

void loop() 
{
  if (digitalRead(int2Pin)==1){    // If int2 goes high, either p/l has changed or there's been a single/double tap
    source = readRegister(MMA8452_ADDRESS, 0x0C);  // Read the interrupt source reg.
    if ((source & 0x08)==0x08) { // We are only interested in the TAP register so read that
      if (millis() >= (LastKnock + Debounce)) {    // Need to debounce the knocks
        Serial.println("movement detected...");
        ledState = !ledState;                        // toggle the status of the ledPin:
        digitalWrite(ledPin, ledState);              // update the LED pin itself
        LastKnock = millis();
        if (millis() >= LastReport + ReportingInterval) {
          if (gps.sentenceAvailable()) gps.parseSentence();
          if (gps.newValuesSinceDataRead()) {
            gps.dataRead();
            Serial.println("logging");
          } 
          //GetConnected();
          if(Send2Ubidots(String(value))) {
            Serial.println(F("Data successfully sent to Ubidots"));
            LastReport = millis();  // Reset the timer
            value++;
          }
          else {
            Serial.println(F("Data not accepted by Ubidots - try again"));
          }
          //GetDisconnected();
        }
      Serial.print("Looking for movement...");
      }
      byte source = readRegister(MMA8452_ADDRESS, 0x22);  // Reads the PULSE_SRC register to reset it
    }   
  } 
}

// Here is where we send the information to Ubidots

boolean Send2Ubidots(String value)
{
  char replybuffer[64];          // this is a large buffer for replies
  int count = 0;
  int complete = 0;
  String var = "";
  String le = "";
  ParseLocation();              // Update the location value from the GPS feed
  var="{\"value\":"+ value + ", \"context\":"+ Location + "}";
  int num=var.length();                                       // How long is the payload
  le=String(num);                                             //this is to calcule the length of var
  // Make a TCP connection to remote host
  Serial.println(F("Sending Data to Ubidots"));
  Adafruit_CC3000_Client client = cc3000.connectTCP(ip, 80);
   if (!client.connected()) {
     Serial.println(F("Error: Could not make a TCP connection"));
   }   
   // Make a HTTP GET request
   client.fastrprint(F("POST /api/v1.6/variables/"));
   client.print(idvariable);
   client.fastrprintln(F("/values HTTP/1.1"));
   client.fastrprintln(F("Content-Type: application/json"));
   client.fastrprint(F("Content-Length: "));
   client.println(le);
   client.fastrprint(F("X-Auth-Token: "));
   client.println(token);
   client.fastrprintln(F("Host: things.ubidots.com"));
   client.println();
   client.println(var);
   client.println();
   // See if Ubidots acknowledges the creation of a new "dot" with a "201" code
   unsigned long commandClock = millis();                      // Start the timeout clock
   while(!complete && millis() <= commandClock + TimeOut)         // Need to give the modem time to complete command 
   {
      while(!client.available() &&  millis() <= commandClock + TimeOut);  // Keep checking to see if we have data coming in
      while (client.available()) {
         replybuffer[count] = client.read();
         count++;
         if(count==63) break;
       }
      Serial.print(F("Reply: "));
      for (int i=0; i < count; i++) {
        if (replybuffer[i] != '\n') Serial.write(replybuffer[i]);
      }
      Serial.println("");                           // Uncomment if needed to debug
      for (int i=0; i < count; i++) {
        if(replybuffer[i]=='2' && replybuffer[i+1]=='0' && replybuffer[i+2] == '1') {  // here is where we parse "201"
          complete = 1;
         break;
        }
      }
    }
  unsigned long CommandTime = millis();
  Serial.print("Closing the socket: ");
  while (!client.close()) {   // Close socket
    if (millis() >= CommandTime + TimeOut) {
      Serial.println("Failed");
    }
  }
  Serial.println("Succeeded");
  if (complete ==1) return 1;            // Returns "True"  if we get the 201 response
  else return 0;
}

// Disconnect from Wifi
boolean GetDisconnected()
{
  unsigned long CommandTime = millis();
  Serial.print("Disconnecting: ");
  while (!cc3000.disconnect() ) {    // Disconnect WiFi
    if (millis() >= CommandTime + TimeOut) {
      Serial.println("Failed");
      return 0;
    }
  }
  Serial.println("Succeeded");
  return 1;
}

 
// Connection to Wifi 
boolean GetConnected()
{
  unsigned long CommandTime = 0;
  Serial.print(F("\nAttempting to connect to ")); Serial.print(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(": Failed!");
    return 0;
  }  
  Serial.println(": Succeeded");
  Serial.print("Request DHCP: ");
  CommandTime = millis();
  while (!cc3000.checkDHCP())    /* Wait for DHCP to complete */
  {
    if (millis() >= CommandTime + TimeOut) {
      Serial.println("Failed");
      return 0;
    }
  }  
  Serial.println("Succeeded");
  return 1;
}
         
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}


boolean ParseLocation() 
// Refer to http://www.gpsinformation.org/dale/nmea.htm#GGA
// Sample data: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
{
  char Latitude[10];
  char Longitude[10];
  float Lat = gps.latitude;
  float Lon = gps.longitude;
  sprintf(Latitude, "%.5f", Lat);
  sprintf(Longitude, "%.5f", Lon);
  Location = "{\"lat\":" + String(Latitude) + ",\"lng\":" + String(Longitude) + "}";
  //Serial.println(Location);
} 

// Initialize the MMA8452 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
// Feel free to modify any values, these are settings that work well for me.
void initMMA8452(byte fsr, byte dataRate)
{
  MMA8452Standby();  // Must be in standby to change registers
  // Set up the full scale range to 2, 4, or 8g.
  if ((fsr==2)||(fsr==4)||(fsr==8))
    writeRegister(0x1D, 0x0E, fsr >> 2);  
  else
    writeRegister(0x1D,0x0E, 0);
  // Setup the 3 data rate bits, from 0 to 7
  writeRegister(0x1D, 0x2A, readRegister(0x1D,0x2A) & ~(0x38));
  if (dataRate <= 7)
    writeRegister(0x1D,0x2A, readRegister(0x1D,0x2A) | (dataRate << 3));   
  /* Set up single and double tap - 5 steps:
   1. Set up single and/or double tap detection on each axis individually.
   2. Set the threshold - minimum required acceleration to cause a tap.
   3. Set the time limit - the maximum time that a tap can be above the threshold
   4. Set the pulse latency - the minimum required time between one pulse and the next
   5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
   for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
  //writeRegister(0x21, 0x7F);  // 1. enable single/double taps on all axes
  writeRegister(0x1D, 0x21, 0x55);  // 1. single taps only on all axes
  // writeRegister(0x21, 0x6A);  // 1. double taps only on all axes
  writeRegister(0x1D, 0x23, Sensitivity);  // 2. x thresh at 2g (0x20), multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(0x1D, 0x24, Sensitivity);  // 2. y thresh at 2g (0x20), multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(0x1D, 0x25, Sensitivity);  // 2. z thresh at .5g (0x08), multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(0x1D, 0x26, 0x30);  // 3. 30ms time limit at 800Hz odr, this is very dependent on data rate, see the app note
  writeRegister(0x1D, 0x27, 0xC8);  // 4. 1000ms (at 100Hz odr, Normal, and LPF Disabled) between taps min, this also depends on the data rate
  writeRegister(0x1D, 0x28, 0xFF);  // 5. 318ms (max value) between taps max
  // Set up interrupt 1 and 2
  writeRegister(0x1D, 0x2C, 0x02);  // Active high, push-pull interrupts
  writeRegister(0x1D, 0x2D, 0x19);  // DRDY, P/L and tap ints enabled
  writeRegister(0x1D, 0x2E, 0x01);  // DRDY on INT1, P/L and taps on INT2
  MMA8452Active();  // Set to active to start reading
}

// Sets the MMA8452 to standby mode.
// It must be in standby to change most register settings
void MMA8452Standby()
{
  byte c = readRegister(0x1D,0x2A);
  writeRegister(0x1D,0x2A, c & ~(0x01));
}

// Sets the MMA8452 to active mode.
// Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(0x1D,0x2A);
  writeRegister(0x1D,0x2A, c | 0x01);
}


// Read a single byte from address and return it as a byte
byte readRegister(int I2CAddress, byte address)
{
  //Send a request
  //Start talking to the device at the specified address
  Wire.beginTransmission(I2CAddress);
  //Send a bit asking for requested register address
  Wire.write(address); 
  //Complete Transmission 
  Wire.endTransmission(false); 
  //Read the register from the device
  //Request 1 Byte from the specified address
  Wire.requestFrom(I2CAddress, 1);
  //wait for response 
  while(Wire.available() == 0);
  // Get the temp and read it into a variable
  byte data = Wire.read(); 
  return data;
}

// Writes a single byte (data) into address
void writeRegister(int I2CAddress, unsigned char address, unsigned char data)
{
  //Send a request
  //Start talking to the device at the specified address
  Wire.beginTransmission(I2CAddress);
  //Send a bit asking for requested register address
  Wire.write(address); 
  Wire.write(data);
  //Complete Transmission 
  Wire.endTransmission(false); 
}

