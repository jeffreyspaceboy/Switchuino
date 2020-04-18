/*************************************************************
  This is a Blynk Example that was modified for the Switchuino.

  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  This example runs directly on ESP32 chip.

  Note: This requires ESP32 support package:
    https://github.com/espressif/arduino-esp32

  Please be sure to select the right ESP32 module
  in the Tools -> Board menu!

  Change WiFi ssid, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!
 *************************************************************/
#include <WiFi.h> //Download
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h> //Download
#include <servo.h> //Download

//#define BLYNK_PRINT Serial /* Comment this out to disable prints and save space */

#define BLINK_COLOR_MAX 1023
#define RGB_COLOR_MAX 10000

//RGB LED Pin Numbers for the Switchuino 
//#define R_LED 25
//#define G_LED 26
//#define B_LED 27
#define R_LED 22
#define G_LED 18
#define B_LED 19
#define RGB_MULTIPLIER 10000

//Mount LED Pin Numbers for the Switchuino (Located on the Servo Mounting Brackets)
#define MOUNT_LED_1 5
#define MOUNT_LED_2 14

uint8_t ledChannel[5] = {1, 2, 3, 4, 5};

//Servo Pin Numbers for Switchuino (NOTE: only SERVO1 will be used initially, you may add the second servo if you'd like)
#define SERVO1 4
Servo myServo1;
//#define SERVO2 16

//Servo State Positions (NOTE: These may need to be changed for each servo)
//ON should be the minimum angle required to turn on the switch consistently.
#define SERVO1_ON 0    /***USER_SHOULD_EDIT***/
//REST should be the angle where the servo bar is parallel to the wall/
#define SERVO1_REST 90 /***USER_SHOULD_EDIT***/
//OFF should be the minimum angle required to turn off the switch consistently.
#define SERVO1_OFF 180 /***USER_SHOULD_EDIT***/

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "YourAuthTokenWillBeAboutThisLong"; /***USER_MUST_EDIT***/

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "YourWiFiName"; /***USER_MUST_EDIT***/
char pass[] = "YourWiFiPassword"; /***USER_MUST_EDIT***/

BLYNK_WRITE(V0) { //Control RGB LED
  for(int i=0; i<3; i++){
    ledcWrite(ledChannel[i], param[i].asInt()*RGB_MULTIPLIER); //Write RGB LED to values indicated by Blynk
  }
}

BLYNK_WRITE(V1) { //Control Mount 1 LED
  ledcWrite(ledChannel[3], param.asInt()); //Write to Mount 1 LED
}

BLYNK_WRITE(V2) { //Control Mount 2 LED
  ledcWrite(ledChannel[4], param.asInt()); //Write to Mount 2 LED
}

bool switchOn = false;
BLYNK_WRITE(V3) { //Toggle Light Switch
  int goTo; //Selected angle to move servo to
  if (param.asInt() == 1 && !switchOn) {
    goTo = SERVO1_ON; //Set 
  } else if (param.asInt() == 0 && switchOn) {
    goTo = SERVO1_OFF;
  }
  myServo1.attach(SERVO1);
  myServo1.write(SERVO1_REST);
  delay(250);
  myServo1.write(goTo);
  delay(400);
  myServo1.write(SERVO1_REST);
  delay(250);
  myServo1.detach();
  switchOn = !switchOn;
}

void setup(){ //Device Setup
  Serial.begin(115200); /*Initialize Serial with 115200 Baud*/
  
  //Attaching each channel to the correct pin
  ledcAttachPin(R_LED, ledChannel[0]);
  ledcAttachPin(G_LED, ledChannel[1]);
  ledcAttachPin(B_LED, ledChannel[2]);
  ledcAttachPin(MOUNT_LED_1, ledChannel[3]);
  ledcAttachPin(MOUNT_LED_2, ledChannel[4]);

  //Initialize LED Channels:
  for(int i=0; i<5; i++){
    ledcSetup(ledChannel[i], 12000, 8); //12 kHz PWM, 8-bit resolution
    ledcWrite(ledChannel[i], 0); //Set all LED's to LOW
  }

  myServo1.attach(SERVO1); //Begins PWM for servo.
  myServo1.write(SERVO1_REST); //Move servo to rest position (between on and off positions).
  delay(300); //Wait for servo movement
  myServo1.detach(); //Turn off PWM signal to servo (saves power and stops any buzzing)
  
  Blynk.begin(auth, ssid, pass); //Setup Wifi and connect with Blynk
}

void loop(){ //Main Loop
  Blynk.run(); 
}
