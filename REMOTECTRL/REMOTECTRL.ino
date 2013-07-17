#include <AFMotor.h>

#include <PS2X_lib.h>  //for v1.6
 
PS2X ps2x;

int error = 0;
byte type = 0;
byte vibrate = 0;
int m1 = 0; 
int m2 = 0; 
int motor1speed = 255;
int motor2speed = 255; 
float StickX;
float StickY;
AF_DCMotor motor1(1); // create motor #1, 1KHz pwm (left motor)
AF_DCMotor motor2(4); // create motor #2, 1KHz pwm (right motor)

void setup() 
{
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");
  
  //Playstation 2 controller code in setup is repurposed from the ps2x library's example
  
  error = ps2x.config_gamepad(13,6,10,9, true, true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
 
  if(error == 0)
  {
    Serial.println("Found Controller, configured successful");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Go to www.billporter.info for updates and to report bugs.");
  }
  else if(error == 1)
  {
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  }
  else if(error == 2)
  {
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  }
  else if(error == 3)
  {
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  }
 
  type = ps2x.readType();
  
  if (type != 1)
  {
    Serial.println("warning: DualShock Controller Not Found!");
  }
}
 
void loop() 
{

 if(error == 1) //skip loop if no controller found
 return;
 
 if (type == 1)
  {
   ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed
 
   INPUT_CONTROLLER();
   OUTPUT_MOTOR();
   Serial.print("Analog Stick Positions: ");
   Serial.print(StickX);
   Serial.print(", ");
   Serial.print(StickY);
   Serial.print(", ");
   Serial.print(m1);
   Serial.print(", ");
   Serial.println(m2);
  }
  else
  {
   Serial.println("STOP");
   motor1.run(RELEASE);
   motor2.run(RELEASE);
  }
  delay(100);
}

////////// Input Functions (Sensors, Wireless Controller) //////////
void INPUT_CONTROLLER() //Interprets the controller's input
{
  StickX = ps2x.Analog(PSS_LX);
  StickY = ps2x.Analog(PSS_LY);  
  StickX = StickX - 128;
  StickX = -StickX;
  StickY = StickY - 128;
  StickY = -StickY;
  m1 = StickY;
  m2 = StickY;
  m1 = m1 - StickX;
  m2 = m2 + StickX;
  
  m1 = m1 * 2;
  m2 = m2 * 2;
  
}

////////// Output Functions (Motor) //////////
void OUTPUT_MOTOR() //regulates speed of motors, direction they are moving
{

  m1 = m1 * 2;
  m2 = m2 * 2;
  if (m1 < 128 && m1 > -128)
  {
   m1 = 0; 
  }
  if (m2 < 128 && m2 > -128)
  {
   m2 = 0; 
  }
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
  
  if(m1 > 255)
  {
   m1 = 255; 
  }
  if (m1 < -255)
  {
   m1 = -255; 
  }
    if(m2 > 255)
  {
   m2 = 255; 
  }
  if (m2 < -255)
  {
   m2 = -255; 
  }
    /// Sets speed ///
  if(m1 > 0)
  {
   motor1.setSpeed(m1);
  }
  else
  {
   motor1.setSpeed(-m1); 
  }
  if(m2 > 0)
  {
   motor2.setSpeed(m2);
  }
  else
  {
   motor2.setSpeed(-m2); 
  }
}
