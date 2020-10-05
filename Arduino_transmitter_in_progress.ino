//===Kart Transmitter Code===//

// include headers
#include <RH_ASK.h>
#include <SPI.h>

// initialize and delcare 

//----------------------Arduino pins----------------------//
const int JOYSTICK_X_PIN = A0;
const int JOYSTICK_Y_PIN = A1;
const int POT_PIN = A2;
const int JOYSTICK_BUT_PIN = 4;
const int LED_PIN = 5;

//-----------------definitions and declarations-------------//

struct joystickValues{
  float ValueX ;
  int ValueY;
  int joyBut;
  float knob;
}myData;

RH_ASK driver(2000, 8, 12);

//-------------------global variables----------------------//
int ledBright = 175;
byte tx_buf[sizeof(myData)] = {0};
unsigned long lastUpdateRadio = 0;
unsigned long lastUpdateLed = 0;
unsigned long lastUpdateX = 0;
int updateJoy = 100;
int updateLed = 1500;
int updateTransmit = 150;


//-------------------MAIN PROGRAM---------------------------//

void setup() 
{
   //Serial.begin(9600);
   pinMode(JOYSTICK_Y_PIN, INPUT);
   pinMode(JOYSTICK_X_PIN, INPUT); 
   pinMode(JOYSTICK_BUT_PIN, INPUT);
   pinMode(POT_PIN, INPUT);
   pinMode(LED_PIN, OUTPUT);
   digitalWrite(JOYSTICK_BUT_PIN, HIGH);
   
   if (!driver.init())
         Serial.println("init failed");
}

void loop() 
{
  
  joystickTel();
  sendToRadio();
  radioLed();
  
}

//--------------------function declarations-----------------//

// Converts x-axis inputs from 8 bit to 10 bit resolution
// output has range from 83 to 113

float mapSteering(float input)
{
  float slope = 1.0 * (113 - 83) / (1023 - 0);
  float output = 83 + slope * (input - 0);
  return output;
}

// Converts y-axis inputs from 8 bit to 10 bit resolution
// output has range from -255 to 255

float mapThrottle(float input)
{
  float slope = 1.0 * (255 - (-255)) / (1023); 
  float output = -255 + slope * (input);
  return output; 
}

// Places joystick inputs into struct

void joystickTel(void)
{ 
  if ((millis() - lastUpdateX) > updateJoy)
  {
    lastUpdateX = millis();
    myData.ValueX = analogRead(JOYSTICK_X_PIN);
    myData.ValueX = mapSteering(myData.ValueX);
    myData.ValueY = analogRead(JOYSTICK_Y_PIN);  
    myData.ValueY = -mapThrottle(myData.ValueY);
    myData.joyBut = digitalRead(JOYSTICK_BUT_PIN);
    myData.knob = analogRead(POT_PIN);
  }

  // ---------debugging---------- //
  // Shows values in struct
  /*
  Serial.print("X value: ");
  Serial.println(myData.ValueX);
  Serial.print("Y value: ");
  Serial.println(myData.ValueY);
  Serial.print("Button: ");
  Serial.println(myData.joyBut);
  Serial.print("Knob: ");
  Serial.println(myData.knob);
  */
}

// Transmits joystick data struct to kart

void sendToRadio(void)
{
  if ((millis() - lastUpdateRadio) > updateTransmit)
  {
    lastUpdateRadio = millis();
    memcpy(tx_buf, &myData, sizeof(myData) );
    byte zize = sizeof(myData);
    driver.send((uint8_t *)tx_buf, zize);
    driver.waitPacketSent();
  }
}

// Does something with transmitter LED

void radioLed(void)
{
  // fading led
  if ((millis() - lastUpdateLed) > updateLed)
  {
    lastUpdateLed = millis();
    if (ledBright == 175)
    {
      ledBright = 0;
      analogWrite(LED_PIN, ledBright);
    }
    else
    {
      ledBright = 175;
      analogWrite(LED_PIN, ledBright);
    }    
  }
}
