//All code used to read the GPS is from the Adafruit GPS Library's examples
//All code used to read the Compass is from this website: http://recombine.net/blog/article/49/hmc6352-sparkfun-compass-and-arduino 

/////****** Destination latitude and longitude (Decimal Minutes) and trim for the compass due to magnetic interference (Degrees) *****/////
float destlat = 38.7643;
float destlon = -122.4284;
float trim = 35;  //to find this value, point the tank North to find out how many degrees off the compass is reading. This value should be positive if the heading is >180, and negative if <180


////////// Ultrasonic //////////
#include "Ultrasonic.h"
Ultrasonic FrontSens(A8,A9);
float UvalFront;
Ultrasonic RightSens(A10,A11);
float UvalRight;
Ultrasonic LeftSens(A12,A13);
float UvalLeft;

////////// GPS //////////
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial lcd (31, 33); //LCD RX is connected to pin 33

// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(18, 19);

//Adafruit_GPS GPS(&mySerial);
// If using hardware serial (e.g. Arduino Mega), comment
// out the above six lines and enable this line instead:
Adafruit_GPS GPS(&Serial1);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

////////// COMPASS //////////
#include <Wire.h>
int HMC6352Address = 0x42;
// This is calculated in the setup() function
int slaveAddress;
int ledPin = 13;
boolean ledState = false;
byte headingData[2];
int i, headingValue;

float headingval;
float curheading;
float gpslat;
float gpslon;
float deltY;
float deltX;

float destang;
boolean destarrive = false;
int isturningL = 0;
int isturningR = 0;
int afterturncount = 0;

float optang;
float heading;
float heading2;

////////// Motor //////////
#include <AFMotor.h>
int m1 = 0; 
int m2 = 0; 
int motor1speed = 255;
int motor2speed = 255;

AF_DCMotor motor1(1); // create motor #1, 1KHz pwm (left motor)
AF_DCMotor motor2(4); // create motor #2, 1KHz pwm (right motor)

void setup()  
{
   lcd.begin(9600);
 pinMode(1, OUTPUT);
 delay(10);
 lcd.write(17);   //Turns Backlight ON
 delay(10);
  ////////// GPS //////////  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  
  ////////// COMPASS //////////
  
  // Shift the device's documented slave address (0x42) 1 bit right
  // This compensates for how the TWI library only wants the
  // 7 most significant bits (with the high bit padded with 0)
  slaveAddress = HMC6352Address >> 1;   // This results in 0x21 as the address to pass to TWI
  pinMode(ledPin, OUTPUT);      // Set the LED pin as output
  Wire.begin();
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
void loop()                     // run over and over again
{
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) 
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  gpslat = GPS.latitude;
  gpslon = GPS.longitude;
  
  INPUT_COMPASS();  //Input compass data
  INPUT_ULTRASONIC();
  CALC_ANGOFF();    //Calculate the direction that the tank needs to go
  CALC_MOTORCTRL(); //Calculate the direction each motor needs
  OUTPUT_SERIAL();  //Outputs debug info to the Serial Monitor (115200 baud)
  OUTPUT_MOTOR();   //Sends signal to motors
}
void INPUT_COMPASS() //Recieves Compass heading
{
  Wire.beginTransmission(slaveAddress);
  Wire.write("A");              // The "Get Data" command
  Wire.endTransmission();
  delay(10);                   // The HMC6352 needs at least a 70us (microsecond) delay
  // after this command.  Using 10ms just makes it safe
  // Read the 2 heading bytes, MSB first
  // The resulting 16bit word is the compass heading in 10th's of a degree
  // For example: a heading of 1345 would be 134.5 degrees
  Wire.requestFrom(slaveAddress, 2);        // Request the 2 byte heading (MSB comes first)
  i = 0;
  while(Wire.available() && i < 2)
  { 
    headingData[i] = Wire.read();
    i++;
  }
  headingValue = headingData[0]*256 + headingData[1];  // Put the MSB and LSB together 
}
void INPUT_ULTRASONIC() //Recieves Ultrasonic data
{
  UvalFront = FrontSens.Ranging(CM);
  delay(10);
  UvalRight = RightSens.Ranging(CM);
  delay(10);
  UvalLeft = LeftSens.Ranging(CM);
  delay(10);
}

void CALC_ANGOFF() //Calculates the direction that the tank needs to face
{
  headingval = headingValue % 10;
  curheading = headingValue / 10;
  curheading = curheading + (headingval /10); 
  curheading = curheading + trim;
  if (GPS.lat == 'S')
  {
    gpslat = -gpslat;
  }
  gpslat = DEGMS_TO_DDEG(gpslat);
  if (GPS.lon == 'W')
  {
    gpslon = -gpslon;
  }
  gpslon = DEGMS_TO_DDEG(gpslon);
  /// convert raw heading to degrees off N ///
  if (curheading > 180)
  {
    curheading = 360 - curheading;
    curheading = -curheading;
  }
  deltY = gpslat - destlat;
  deltX = gpslon - destlon;
  destang = atan(deltX / deltY);
  optang = ((destang / 3.14159265) * 180) + curheading;
}
void CALC_MOTORCTRL() //Motor control logic -- Integrates GPS/Compass data and Ultrasonic data, to allow the tank to avoid obstacles and move to a destination
{
  m1 = 0;
  m2 = 0;
  
  if (gpslat <= destlat + 0.00005 && gpslat >= destlat - 0.00005 && gpslon <= destlon + 0.00005 && gpslon >= destlon - 0.00005)
  {
    destarrive = true; 
  }
  else
  {
    destarrive = false; 
  }
  if (GPS.fix && destarrive == false)
  {
      isturningL = 0;
      isturningR = 0;
     
    if (UvalFront < 20 || UvalLeft < 20 || UvalRight < 20 || afterturncount < 0)   //after turning, the tank will go straight for a short amount of time
    {                                                                              //unless the ultrasonic sensors detect an object
      afterturncount = 0;
    } 
    if (afterturncount > 0)
    {
      afterturncount = afterturncount - 1; 
    }
    if (afterturncount == 0)
    {
      if (optang < 45 && optang > -45)
      {
        m1 = 1;
        m2 = 1; 
      }
      if (UvalFront < 20 || UvalLeft < 20 || UvalRight < 20) //object avoidance logic
      {
        if (UvalLeft < 20 && isturningL == 0)
        {
          isturningR = 1; 
        }
        else if(UvalRight < 20 && isturningR == 0)
        {
          isturningL = 1; 
        }
        
        if (UvalLeft < UvalRight && isturningL == 0)
        {
          isturningR = 2; 
        }
        else if (UvalLeft > UvalRight && isturningR == 0)
        {
          isturningL = 2; 
        }
        afterturncount = 30;
      }
      if (UvalFront > 20 && UvalLeft > 20 && UvalRight > 20) // GPS/Compass navigation logic
      {
        if (optang > 10 && isturningR == 0)
        {
          isturningL = 1;
        }
        else if (optang < -10 && isturningL == 0)
        {
          isturningR = 1;
        }
      } 
    }
    m1 = 1;
    m2 = 1;
    if (isturningL == 1)
    {
      m1 = 0;
      m2 = 1;
    }
    if (isturningR == 1)
    {
      m1 = 1;
      m2 = 0;
    }
    if (isturningL == 2)
    {
      m1 = -1;
      m2 = 1;
    }
    if (isturningR == 2)
    {
      m1 = 1;
      m2 = -1;
    }
  }
}
void OUTPUT_SERIAL() //outputs to Serial Monitor (Computer) and the LCD (on the tank)
{
  if (timer > millis())  timer = millis();
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) 
  { 
    timer = millis(); // reset the timer   
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.println((int)GPS.fix);
    
    ////////// Printing the output //////////
    Serial.print("Location: ");
    Serial.print(", ");
    Serial.print(gpslat, 5);
    Serial.print(", ");
    Serial.println(gpslon, 5); 
    Serial.print("Current heading: ");
    Serial.print(int (headingValue / 10));     // The whole number part of the heading 
    Serial.print(".");
    Serial.print(int (headingValue % 10));     // The fractional part of the heading
    Serial.println(" degrees");
    Serial.print("Angle off Optimal Heading: ");
    Serial.print(optang, 1);
    Serial.println(" degrees");
    Serial.print("Ultrasonic Distance (F,L,R)");
    Serial.print(UvalFront);
    Serial.print(",");
    Serial.print(UvalLeft);
    Serial.print(",");
    Serial.println(UvalRight);
    Serial.print(isturningR);
    Serial.println(isturningL);
    
    lcd.write(12);  //Clears Display
    delay(10);
    lcd.write(161);
    lcd.print(UvalFront);
    lcd.write(128);
    lcd.write("H:");
    heading2 = headingValue % 10;
    heading = headingValue / 10;
    heading = heading + (heading2 /10); 
    heading = heading + trim;
    lcd.print(optang, 1);   //Hello CoytHV
    lcd.write(148);
    lcd.print(gpslat);
    lcd.write(",");
    lcd.print(gpslon);
    lcd.write(142);
    if (m1 > 0)
    {
      ForwardArrow();
    }
    if (m1 < 0)
    {
      BackArrow();
    }
    if (m1 == 0)
    {
      NoArrow(); 
    }
    lcd.write(143);
    if (m2 > 0)
    {
      ForwardArrow();
    }
    if (m2 < 0)
    {
      BackArrow();
    }
    if (m2 == 0)
    {
      NoArrow(); 
    }
    
    if (destarrive == true)
    {
      Serial.println("Arrived");
      lcd.write(136);
      lcd.print("ARVD");
    }
    else
    {
      Serial.println("Driving"); 
      lcd.write(136);
      lcd.print("DRVN");
    }
  }
}
void OUTPUT_MOTOR() //regulates speed of motors, direction they are moving
{
  /// Sets speed ///
  motor1.setSpeed(motor1speed);
  motor2.setSpeed(motor2speed);
  /// Controls motor direction ///
  if (m1 >= 1)
  {
    motor1.run(FORWARD);
  }
  if (m1 == 0)
  {
    motor1.run(RELEASE);
  }
  if (m1 <= -1)
  {
    motor1.run(BACKWARD);
  }
  if (m2 >= 1)
  {
    motor2.run(FORWARD);
  }
  if (m2 == 0)
  {
    motor2.run(RELEASE);
  }
  if (m2 <= -1)
  {
    motor2.run(BACKWARD);
  } 
}
float DEGMS_TO_DDEG(float b) //Converts the GPS's output (Degrees, then Decimal Minutes to a more easily usable format (Decimal Degrees)
{
  int a = b / 100;
  b = b - (a * 100);
  b = b / 60;
  b = b + a;
  return b;
}
void ForwardArrow()  //Displays an "up" arrow, indicating that the tread is moving forward
{
  delay(10); 
  lcd.write(253);  //Define Custom Character 5
  lcd.write(B00100);  //8 byte display data follows 
  lcd.write(B01110);
  lcd.write(B11111);
  lcd.write(B00100);
  lcd.write(B00100);
  lcd.write(B00100);
  lcd.write(B00100);
  lcd.write(B00100);
  lcd.write(5);  //Display custom character 5
}
void BackArrow()  //Displays a "down" arrow, indicating that the tread is moving backward
{
  delay(10); 
  lcd.write(252);  //Define Custom Character 5
  lcd.write(B00100);  //8 byte display data follows 
  lcd.write(B00100);
  lcd.write(B00100);
  lcd.write(B00100);
  lcd.write(B00100);
  lcd.write(B11111);
  lcd.write(B01110);
  lcd.write(B00100);
  lcd.write(4);  //Display custom character 5
}
void NoArrow()  //Displays a line, indicating that the tread is not moving
{
  delay(10); 
  lcd.write(252);  //Define Custom Character 5
  lcd.write(B00100);  //8 byte display data follows 
  lcd.write(B00100);
  lcd.write(B00100);
  lcd.write(B00100);
  lcd.write(B00100);
  lcd.write(B00100);
  lcd.write(B00100);
  lcd.write(B00100);
  lcd.write(4);  //Display custom character 5
}
