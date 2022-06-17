/*
 * Include all necessary libraries
 */
#include <Servo.h>                    // Servo library
#include <SoftwareSerial.h>           // DF Player Serial Communication Library
#include <DFRobotDFPlayerMini.h>      // DF Player Control Library
#include <Adafruit_NeoPixel.h>

/* 
 *  Define Arduino Pins
 */
//Use pin 4 for extra 5 volts
#define VCC2 4
// Use A0 Pin for the Force Sensor
#define FORCE_SENSOR_PIN A0           // the FSR and 10K pulldown are connected to A0

// Flora NeoPixel v2 Pins
#define PIN1 5                         // Flora connected to pin 5
#define PIN2 9                         // Flora connected to pin 9
#define PIN3 10                        // Flora connected to pin 10
#define PIN4 11                        // Flora connected to pin 11

// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 8;                     // Connects to module's RX 
static const uint8_t PIN_MP3_RX = 7;                     // Connects to module's TX 
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);    // Starts DF Player Serial Communication
DFRobotDFPlayerMini player;                               // Create the Player object

int heartVibration = 2;               // Connects to vibration motor for heart
                     
Adafruit_NeoPixel led1 = Adafruit_NeoPixel(1, PIN1); // Connects to first flora
Adafruit_NeoPixel led2 = Adafruit_NeoPixel(1, PIN2); // Connects to second flora
Adafruit_NeoPixel led3 = Adafruit_NeoPixel(1, PIN3); // Connects to third flora
Adafruit_NeoPixel led4 = Adafruit_NeoPixel(1, PIN4); // Connects to fourth flora
unsigned long untouchedStart = 0; // Millis time tracker for flora
int untouchedTime = 5000; // After untouchedTime ms, start floras and barking 
bool untouched = true; // If dog hasn't been touched for more than untouchedTime
bool barked = false; // If dog has barked

// Define Tail Servo Object
Servo tailServo;
// Define Neck Servo Object
Servo neckServo;

/* 
 *  Parameters
 */
// Tail Wagging Parameters
int wag_deg = 30;           // Wag Degree, Right Position > 88
int tail_center = 88;       // Tail Center Position
int wag_speed = 250;        // Wag Delay (Increase Delay, Decrease Speed)

// Head Nudging Parameters
int nudge_deg = 30;         // Nudge Down Position, Down Position > 100
int head_center = 100;      // Head Center Position
int nudge_speed = 500;      // Nudge Delay (Increase Delay, Decrease Speed)

// Touch Sensor Parameters
int max_noTouchCount = 2;   // Maximum Number of No Touch Counts

// Initialization
int noTouchCount = 0;      // Count number of no touches
bool start = true;         // Start touch status
int wag_direction = 1;     // Wag direction

void setup() {
  pinMode(VCC2, OUTPUT); //For extra 5 volts
  digitalWrite(VCC2, HIGH); //For extra 5 volts
  
  // Start Serial Communication
  Serial.begin(9600);

  pinMode(heartVibration, OUTPUT);

  // Start and set brightness for all Floras
  led1.begin();
  led1.setBrightness(127);
  led2.begin();
  led2.setBrightness(127);
  led3.begin();
  led3.setBrightness(127);
  led4.begin();
  led4.setBrightness(127);
  led1.setPixelColor(0, 0, 0, 0); // Reset all floras to start with no color
  led2.setPixelColor(0, 0, 0, 0); 
  led3.setPixelColor(0, 0, 0, 0); 
  led4.setPixelColor(0, 0, 0, 0);  
  led1.show();
  led2.show();
  led3.show();
  led4.show();

  untouchedStart = millis(); // Start timer for floras

  // Attach to tail servo and center the tail.
  tailServo.attach(3);                
  Serial.println("Tail Servo is connected. Centering the tail...");
  tailServo.write(tail_center);
  delay(wag_speed);
  // Attach to neck servo and center the head.
  neckServo.attach(6);     
  Serial.println("Neck Servo is connected. Centering the head...");
  neckServo.write(head_center);
  delay(nudge_speed);

  // Start communication with DFPlayer Mini
  softwareSerial.begin(9600);
  if (player.begin(softwareSerial)) {   
    Serial.println("OK");                                   // Print OK
    player.volume(20);                                      // Set volume to 20 (0 to 30)
  } else {
    Serial.println("Connecting to DFPlayer Mini failed!");  // Print Failed
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  //--------------------------------------------

  if(untouched && (millis() - untouchedStart >= untouchedTime)){
     Serial.println("Lights ON");
     led1.setPixelColor(0, 66, 176, 245); // If hasn't been petted in the past 5 seconds, set lights 
     led2.setPixelColor(0, 245, 66, 173); 
     led3.setPixelColor(0, 245, 215, 66); 
     led4.setPixelColor(0, 66, 245, 182);  
     if(!barked){ // If the dog hasn't barked 
      player.play(1);
      barked = true;   
     }
    /* Continuously play after sound finishes
    if(player.readState() != 513)
    {
      player.play(1);
    }*/
  }else{
     Serial.println("Lights OFF");
     led1.setPixelColor(0, 0, 0, 0); // Otherwise, turn off all lights
     led2.setPixelColor(0, 0, 0, 0); 
     led3.setPixelColor(0, 0, 0, 0);  
     led4.setPixelColor(0, 0, 0, 0); 
     barked = false;
  }
  led1.show();
  led2.show();
  led3.show();
  led4.show();
  
  digitalWrite(heartVibration, HIGH); //vibrate
  delay(250);
  digitalWrite(heartVibration, LOW); //vibrate
  delay(100);
  digitalWrite(heartVibration, HIGH); //vibrate
  delay(250);
  digitalWrite(heartVibration, LOW); //vibrate
  delay(250);
    
  // Read Force Sensor Input Value
  int force = analogRead(FORCE_SENSOR_PIN);
  Serial.println(force);
  
  // If the FS input is more than 50 or (Start Status is False and noTouchCount is less than 5)
  if(force > 200 || (!start && noTouchCount < max_noTouchCount))
  {  
    if(start)
    {
      tailServo.write(tail_center + wag_direction * wag_deg); // Wag
      delay(wag_speed);
      neckServo.write(head_center + wag_direction * nudge_deg);  // Nudge Up
      delay(nudge_speed);
      start = false;              // Start status set to false because it is already started.
    }
    else
    {
      // If stop petting
      if(force <= 200)
      {
        // Start incrementing no touch count
         noTouchCount++;
      }
      else
      {
        // Reset the no touch count
        noTouchCount = 0;
      }
      tailServo.write(tail_center + wag_direction * wag_deg);    // Wag
      delay(wag_speed);
      neckServo.write(head_center + wag_direction * nudge_deg);  // Nudge
      delay(nudge_speed);
    }
  
    wag_direction *= -1;
    untouched = false;
  }
  else
  {
    if(!start)
    {
      tailServo.write(tail_center + wag_direction * wag_deg); // Wag 
      delay(wag_speed);
      neckServo.write(head_center + wag_direction * nudge_deg); // Nudge 
      delay(nudge_speed);
      start = true;
      untouched = true;
      untouchedStart = millis();
    }
    else
    {
      neckServo.write(head_center);    // Neck stills   
      delay(nudge_speed);
      tailServo.write(tail_center);    // Stop Wagging
      delay(wag_speed);
    }
    noTouchCount = 0;
  }
}
