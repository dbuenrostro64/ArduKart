//===KART CODE===//

// include headers
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <Servo.h>
#include <RH_ASK.h>
#include <Adafruit_SSD1306.h>
#include <NewPing.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_L3GD20.h>

//-----------------DEFINITIONS---------------------------//

//------OLED display----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1

//-----ultrasonic sensor------
#define MAX_DISTANCE 200

// initialize and declare 

//----------------------Arduino pins----------------------//

//------kart movement pins---------
const int MOTOR_CON_PIN = 6;  
const int MOTOR_F_PIN = 7;
const int MOTOR_R_PIN = 11;
const int SERVO_DATA_PIN = 9;
//------telemetry pins-------------
const int RX_PIN = 8;
// ultrasonic sensor
const int ULTRA_TRIG_PIN = 12;
const int ULTRA_ECHO_PIN = 13;
// accelerometer
const int ACC_SCL_PIN = A0;
const int ACC_SDA_PIN = A1;
const int ACC_SDO_PIN = A2;
const int ACC_CS_PIN = A3;
// gyroscope
const int GYRO_SCL_PIN = 2;
const int GYRO_SDA_PIN = 3;
const int GYRO_SA0_PIN = 4;
const int GYRO_CS_PIN = 5;

//-----------------global variables---------------------//

//------kart movement variables--------
const int kartForward = 98; // going straight ahead
const int kartLeft = 88; // farthest left turn
const int kartRight = 118; // farthest right turn
int motorSpeed = 0; //starts at 0 mph
int halfSpeed = 175;
//------other kart component vars---------
long sonicDuration;
int sonicDistance;
int sonicTemp;
unsigned long lastUpdateRx = 0;
unsigned long lastUpdateServo = 0;
unsigned long lastUpdateUltrasonic = 0;
unsigned long updateRate = 100;
//------time values for gyro calc---------
float currentTime = 0;
float elapsedTime = 0;
float previousTime = 0;
//------acc and gyro vars-----------
float accAngleX, accAngleY;
float gyroAngleX, gyroAngleY, gyroAngleZ;
float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
float roll, pitch, yaw;
int c = 0;

//-----------------definitions and declarations-------------//

// function prototypes
bool isReceiveFromRadio();
void kartControls(void);
int displayOnScreen(int data);
String textOnScreen(String data);
int ultrasonicData();
int accelerometerData(void);
int gyroData(void);
void findAccError(void);
void complementaryFilter(void);
void autonomousForward(int miliseconds);
void autonomousReverse(int miliseconds);
void autonomousTurn(bool turnLeft, int miliseconds);
void autoTurn90(String direction);
void activateAuto(void);
void findGyroError(void);

// struct def which receives input variables from radio
struct joystickValues{
  float ValueX;
  int ValueY;
  int joyBut;
  float knob;
}joystick;

// create object for servo motor
Servo Steering;

// create object for receiver(speed,rxPin,txPin)
RH_ASK driver(2000, RX_PIN, 12);

// create object for oled display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// create object for ultrasonic sensor
NewPing sonar(ULTRA_TRIG_PIN, ULTRA_ECHO_PIN, MAX_DISTANCE);

// create object for acc and gyro controls
sensors_event_t event;

// create object for accelerometer
Adafruit_LIS3DH lis = Adafruit_LIS3DH(ACC_CS_PIN, ACC_SDA_PIN, ACC_SDO_PIN, ACC_SCL_PIN);

// create object for gyroscope
Adafruit_L3GD20 gyro(GYRO_CS_PIN, GYRO_SA0_PIN, GYRO_SDA_PIN, GYRO_SCL_PIN);

//-------------------MAIN PROGRAM---------------------------//

void setup() 
{
  pinMode(motorSpeed, OUTPUT); //delete?
  pinMode(MOTOR_CON_PIN, OUTPUT);
  pinMode(MOTOR_F_PIN, OUTPUT);
  pinMode(MOTOR_R_PIN, OUTPUT);
  pinMode(ULTRA_TRIG_PIN, OUTPUT); 
  pinMode(ULTRA_ECHO_PIN, INPUT); 
  digitalWrite(MOTOR_F_PIN, LOW);
  digitalWrite(MOTOR_R_PIN, LOW);
  
  Serial.begin(9600);
  Steering.attach(SERVO_DATA_PIN);
   if (!driver.init())
    Serial.println("init failed");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    Serial.println("SSD1306 allocation failed");

  if (! lis.begin(0x18))    
    Serial.println("Couldnt start");

  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
    Serial.println("gyro init failed");

  lis.setRange(LIS3DH_RANGE_4_G);
  
  display.display();
  delay(300);
  display.clearDisplay();
  //textOnScreen("AYYLMAO");
  findGyroError();
  findAccError();
  autonomousForward(1500);
  autoTurn90("left");
  autonomousForward(1500);
  autoTurn90("right");
  autonomousForward(1500);
}

void loop() 
{
  //accelerometerData();
  //gyroData();
  //complementaryFilter();
//  activateAuto();
//  sonicTemp = ultrasonicData();
//  displayOnScreen(sonicTemp);
// 
  if (isReceiveFromRadio())
  {
    kartControls();
    //activateAuto();
  }
  
  
}
//-----------------function declarations------------------//

// Receives movement struct from radio transmitter

bool isReceiveFromRadio()
{
  // if the time since last update is greater than the update frequency
  if ((millis() - lastUpdateRx) > updateRate)  
  {
    // last update is now
    lastUpdateRx = millis();
    uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);

    // if message is availible, copy to buf
    if(driver.recv(buf, &buflen))
    {
      // copies from buffer to data structure
      memcpy(&joystick, buf, sizeof(joystick));

      // -----------debugging------------ //
      // shows joystick values recieved
      
//      Serial.print("X value: ");
//      Serial.println(joystick.ValueX);
//      Serial.print("Y value: ");
//      Serial.println(joystick.ValueY);
//      Serial.print("Button: ");
//      Serial.println(joystick.joyBut);
//      Serial.print("Knob: ");
//      Serial.println(joystick.knob);
      
      return true;
    }
  }
  return false;
}

// Moves kart using movement struct

void kartControls(void)
{
  // servo moves to position sent by Tx
  Steering.write(joystick.ValueX);

  // motor speed depends on joystick position
  motorSpeed = joystick.ValueY;
      
  //Serial.print("speed: ");
  //Serial.println(motorSpeed);

  // no movement
  if (joystick.ValueY >= -20 && joystick.ValueY <= 20)
  {
      analogWrite(MOTOR_CON_PIN, motorSpeed);
      digitalWrite(MOTOR_F_PIN, LOW);
      digitalWrite(MOTOR_R_PIN, LOW);
  }

  // move forward
  else if (motorSpeed >= 0)
  {
    analogWrite(MOTOR_CON_PIN, motorSpeed);
    digitalWrite(MOTOR_F_PIN, HIGH);
    digitalWrite(MOTOR_R_PIN, LOW);
  }

  // move backwards
  else if (motorSpeed <= 0)
  {
    motorSpeed = -motorSpeed;
    analogWrite(MOTOR_CON_PIN, motorSpeed);
    digitalWrite(MOTOR_F_PIN, LOW);
    digitalWrite(MOTOR_R_PIN, HIGH);
  }
  
}

// Displays variable that was passed on OLED display

int displayOnScreen(int data)
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.cp437(true);
  display.println("COUNTER");
  display.print("NUM: ");
  display.println(data);
  display.display();
  
}

// Displays string that was passed on OLED display

String textOnScreen(String data)
{
  display.clearDisplay();
  display.setTextSize(2.5);
  display.setTextColor(WHITE);
  display.setCursor(0,10);
  display.cp437(true);
  display.println(data);
  //display.startscrollleft(0x00, 0x0F);
  display.display();
  
}

// Returns distance from ultrasonic sensor

int ultrasonicData()
{
  if ((millis() - lastUpdateUltrasonic) > updateRate) 
    return (sonar.ping_cm());
    
}

// returns values from accelerometer

int accelerometerData(void)
{
  lis.getEvent(&event);
  // angle calc
  accAngleX = (atan(event.acceleration.y / sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.z, 2))) * 180 / PI) - accErrorX ;
  accAngleY = (atan(-1 * event.acceleration.x / sqrt(pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2))) * 180 / PI) - accErrorY ;

  // -----------debugging------------ //
  // shows accelerometer values recieved
//  Serial.print("X:  "); Serial.print(event.acceleration.x); 
//  Serial.print(" Y:  "); Serial.print(event.acceleration.y); 
//  Serial.print(" Z:  "); Serial.print(event.acceleration.z); Serial.println(" m/s^2");
//  Serial.print("X: "); Serial.print(accAngleX);
//  Serial.print(" Y: "); Serial.println(accAngleY);

}

// returns values from gyroscope

int gyroData(void)
{
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;
  gyro.read();
  //error correction
  gyro.data.x = gyro.data.x - gyroErrorX;
  gyro.data.y = gyro.data.y - gyroErrorY;
  gyro.data.z = gyro.data.z - gyroErrorZ;
  //angle calc
  gyroAngleX = gyroAngleX + gyro.data.x * elapsedTime;
  gyroAngleY = gyroAngleY + gyro.data.y * elapsedTime;
  yaw = yaw + gyro.data.z * elapsedTime;
  
  // -----------debugging------------ //
  // shows gyroscope values recieved
//  Serial.print("X: "); Serial.print((int)gyro.data.x);   
//  Serial.print(" Y: "); Serial.print((int)gyro.data.y);   
//  Serial.print(" Z: "); Serial.println((int)gyro.data.z); 
//  Serial.print("X: "); Serial.print(gyroAngleX);
//  Serial.print(" Y: "); Serial.print(gyroAngleY);
  Serial.print(" Z: "); Serial.println(yaw);  
  
}

void findAccError(void)
{
  c = 0;
  while(c < 350)
  {
    lis.getEvent(&event);
    accErrorX = accErrorX + ((atan(event.acceleration.y / sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.z, 2))) * 180 / PI));
    accErrorY = accErrorY + ((atan(-1 * event.acceleration.x / sqrt(pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2))) * 180 / PI));
    c++;
  }
  accErrorX = accErrorX / 350;
  accErrorY = accErrorY / 350;

  // -----------debugging------------ //
//  Serial.print("acc-X-Error "); Serial.print(accErrorX);
//  Serial.print("acc-Y-Error "); Serial.print(accErrorY);
  
}

void findGyroError(void)
{
  c = 0;
  while(c < 350)
  {
    gyro.read();
    gyroErrorX = gyroErrorX + gyro.data.x;
    gyroErrorY = gyroErrorY + gyro.data.y;
    gyroErrorZ = gyroErrorZ + gyro.data.z;
    c++;
  }
  gyroErrorX = gyroErrorX / 350;
  gyroErrorY = gyroErrorY / 350;
  gyroErrorZ = gyroErrorZ / 350;
  
  // -----------debugging------------ //
//  Serial.print("gyro-X-Error "); Serial.print(gyroErrorX);
//  Serial.print(" gyro-Y-Error "); Serial.print(gyroErrorY);
//  Serial.print(" gyro-Z-Error "); Serial.println(gyroErrorZ);

}

// combine acc and gyro readings
void complementaryFilter(void)
{
  roll = 0.92 * gyroAngleX + 0.08 * accAngleX;
  pitch = 0.92 * gyroAngleY + 0.08 * accAngleY;
  // -----------debugging------------ //  
//  Serial.print("Roll: "); Serial.print(roll);
//  Serial.print(" Pitch: "); Serial.println(pitch);
  
}

// tells kart to go forward for specified time

void autonomousForward(int miliseconds)
{
  Steering.write(kartForward);
  analogWrite(MOTOR_CON_PIN, halfSpeed);
  digitalWrite(MOTOR_F_PIN, HIGH);
  digitalWrite(MOTOR_R_PIN, LOW);
  delay(miliseconds);
  digitalWrite(MOTOR_F_PIN, LOW);
  digitalWrite(MOTOR_R_PIN, LOW);
  delay(100);    
}

// tells kart to go in reverse for specified time

void autonomousReverse(int miliseconds)
{
  Steering.write(kartForward);
  analogWrite(MOTOR_CON_PIN, halfSpeed);
  digitalWrite(MOTOR_F_PIN, LOW);
  digitalWrite(MOTOR_R_PIN, HIGH);
  delay(miliseconds);
  digitalWrite(MOTOR_F_PIN, LOW);
  digitalWrite(MOTOR_R_PIN, LOW);
  
}

// tells kart to turn left or right for specified time

void autonomousTurn(bool turnLeft, int miliseconds)
{
  if( turnLeft == true)
    Steering.write(kartLeft); 
  
  else
    Steering.write(kartRight);  
    
  analogWrite(MOTOR_CON_PIN, halfSpeed);
  digitalWrite(MOTOR_F_PIN, HIGH);
  digitalWrite(MOTOR_R_PIN, LOW);
  delay(miliseconds);
  digitalWrite(MOTOR_F_PIN, LOW);
  digitalWrite(MOTOR_R_PIN, LOW);
  
}

// turn left or right 90 degrees

void autoTurn90(String direction)
{
  if(direction == "left")
  {
    Steering.write(kartLeft); 
    analogWrite(MOTOR_CON_PIN, halfSpeed);
    while(yaw<90)
    {
    accelerometerData();
    gyroData();
    complementaryFilter();
    digitalWrite(MOTOR_F_PIN, HIGH);
    digitalWrite(MOTOR_R_PIN, LOW);
    }  
  }
  else if (direction == "right")
  {
    Steering.write(kartRight);  
    analogWrite(MOTOR_CON_PIN, halfSpeed);
    while(yaw>-90)
    {
    accelerometerData();
    gyroData();
    complementaryFilter();
    digitalWrite(MOTOR_F_PIN, HIGH);
    digitalWrite(MOTOR_R_PIN, LOW);
    }  
  }
    digitalWrite(MOTOR_F_PIN, LOW);
    digitalWrite(MOTOR_R_PIN, LOW);
    yaw = 0;
    Steering.write(kartForward);
    delay(100);
}

// activate the autonomous routine

void activateAuto(void)
{
  if(joystick.joyBut == 0)
    autonomousForward(2000);
    //autoTurn90(true);
    
}
