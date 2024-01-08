// >>>>>> used libraries <<<<<<

#define BLYNK_PRINT Serial // It is just a Redirection of built in Blynk to serial monitor
#include <BlynkSimpleShieldEsp8266.h> // connect to server of blynk through (auth + esp2866) , show on serial monitor " Blynk"

#include <ESP8266_Lib.h> // for ESP8266 wi-fi module

#include <Servo.h>  // for Servo Motor

#include "TinyGPS++.h" // for Ublox NEO-6m GPS Module

TinyGPSPlus gps; // define gps variable

  int times,dur; // use it in (for loop) for GPS function
  
  float lat ; // latitude variable
  float lon ; // longitude variable
  
  int flag=0 ; // use with notification !!
  
  WidgetMap myMap(V7); // >>>>>> widget on mobile application to show Maps , recive inputs from v7 <<<<<<

Servo myservo;  // create servo object to control a servo

char auth[] = "*****************************************"; // >>>> connection adress between application and arduino through IOT <<<<

// >>>>> Wi-Fi Netwrok <<<<<

char ssid[] = "******";     // network name
char pass[] = "*********"; // network password

int pos=0; // variable for Servo motor postion


#define EspSerial Serial3    // define ESP2866 on Serial 3 of Arduino Mega , EspSerial = serial 3

#define ESP8266_BAUD 115200  // Speed (baud rate) which used to show comments of Serial monitor

 int metal=5;     // metal sensor signal pin
 int metal_val;  // metal sensor variable for blynk
 
ESP8266 wifi(&EspSerial); // Serial communication between arduino to show connected or not , RX + TX to serial 3 , define serial for ESP8266

// >>>>> drivers pins <<<<

// forwad
int m1AR=22; // front right
int m1BR=23; // back  right
int m2AR=24; // front left
int m2BR=25; // back left

// reverse 

int m1AL=26; // front right
int m1BL=27; // back  right
int m2AL=28; // front left
int m2BL=29; // front left

BlynkTimer timer; // repet variable of robot

BLYNK_CONNECTED()
  {
    // >>>> Virtual pin <<<<<
    
    Blynk.syncVirtual(V0);    //fowrwad
    Blynk.syncVirtual(V1);    //right
    Blynk.syncVirtual(V2);    //left
    Blynk.syncVirtual(V3);    //reverse
    
    Blynk.syncVirtual(V4);    //right camera servo
    Blynk.syncVirtual(V6);    //left camera  servo

    Blynk.syncVirtual(V11);    //GPS
  }

   // >>>>>>> Functions of robot <<<<<<< // 
   
  // >>>> Servo Control <<<<<<
  
 BLYNK_WRITE(V4) // servo righ funtion
   {
  int Rc=param.asInt(); // read data from virtual pin and convert to  variable "RC"
 
   if(Rc==1 && pos<180)  // loop to reatch postion
      {
      Serial.println("RC"); // right camera
       pos=pos+10;         // steps
      myservo.write(pos); // send data to servo
      delay(100);
       Rc=0;           // change state of button & prevent vibration for servo
      }
    }
    
 BLYNK_WRITE(V6) // servo left funtion
   {
  int Lc=param.asInt(); // read from virtual pin and save in variable "LC"
  
       if(Lc==1 && pos>0) // loop to reatch postion
    {
      
      Serial.println("LC");
      pos=pos-10;            // left camera
      myservo.write(pos);   // steps
      delay(100);
       Lc=0;              // change state of button & prevent vibration for servo
    }
  } 

  // >>>> end of Servo <<<<<
  
 //************************************************
 
  // >>>>> Robot Control <<<<<

 BLYNK_WRITE(V0) // forwad function
{
  int fwd=param.asInt(); // read from virtual pin and convert to  variable
  Serial.println("fwd");  // print forward
    Serial.println(fwd); // print value of button
    if(fwd==1)
    {
    digitalWrite(m1AR,HIGH); digitalWrite(m1BR,LOW);
     digitalWrite(m2AR,HIGH); digitalWrite(m2BR,LOW);

      digitalWrite(m1AL,HIGH); digitalWrite(m1BL,LOW);
     digitalWrite(m2AL,HIGH); digitalWrite(m2BL,LOW);
    }
    else  stopp(); // stop robot
    
} 
 BLYNK_WRITE(V1) // right funtion
{
  int right=param.asInt(); // read from virtual pin and convert to  variable "right"
  Serial.println("right"); // print right
    Serial.println(right); // print value of button
        if(right==1)
    {
    digitalWrite(m1AR,LOW);digitalWrite(m1BR,HIGH);
     digitalWrite(m2AR,LOW);digitalWrite(m2BR,HIGH);

      digitalWrite(m1AL,HIGH);digitalWrite(m1BL,LOW);
     digitalWrite(m2AL,HIGH);digitalWrite(m2BL,LOW);
    }
    else  stopp();
} 

 BLYNK_WRITE(V2) // left funtion
{
  int left=param.asInt(); // read from virtual pin and convert to  variable "left"
  Serial.println("left"); // print left
    Serial.println(left);// print value of button
           if(left==1)
    {
    digitalWrite(m1AR,HIGH);digitalWrite(m1BR,LOW);
     digitalWrite(m2AR,HIGH);digitalWrite(m2BR,LOW);

      digitalWrite(m1AL,LOW);digitalWrite(m1BL,HIGH);
     digitalWrite(m2AL,LOW);digitalWrite(m2BL,HIGH);
    }
    else  stopp();
    
} 
 BLYNK_WRITE(V3) // reverse function 
{
  int back=param.asInt(); read from virtual pin and convert to  variable "back"
  Serial.println("back"); // print back
    Serial.println(back); // print value of button
     if(back==1)
    {
    digitalWrite(m1AR,LOW);digitalWrite(m1BR,HIGH);
     digitalWrite(m2AR,LOW);digitalWrite(m2BR,HIGH);

      digitalWrite(m1AL,LOW);digitalWrite(m1BL,HIGH);
     digitalWrite(m2AL,LOW);digitalWrite(m2BL,HIGH);
    }
    else  stopp();
} 
void stopp() // to stop robot from moving
  {
  digitalWrite(m1AR,LOW);digitalWrite(m1BR,LOW);
   digitalWrite(m2AR,LOW);digitalWrite(m2BR,LOW);

      digitalWrite(m1AL,LOW);digitalWrite(m1BL,LOW);
     digitalWrite(m2AL,LOW);digitalWrite(m2BL,LOW);
  
  }

  BLYNK_WRITE(V11) // GPS button funtion
   {
       int GPS_btn=param.asInt(); // read from virtual pin and convert to  variable "GPS_btn"
 
       if(GPS_btn==1)  //if gps button pressed
            {
        
             Serial.println("gps button pressed");
             GPS();  
      
            }
    }

 // >>>>>> end of robot control <<<<<<

 // *************************************************

 // >>>>>>>>>>> main function of robot <<<<<<<<<<<<
 
void robot() 
{
  metalSensor();
}

void metalSensor() // metal sensor function
{
  metal_val=digitalRead(metal); // red from sensor
  WidgetLED led1(V5); // define led indecator virtual led
  
   if(metal_val==1)  // >>>found mine <<<
   {
    
    led1.on();     // led ON
    
    flagg++;     // flag=1
    if(flag==1)
    {
      Blynk.notify("Mine is found"); // notification on mobile application
    }
    
    }
    
   else // metal sensor
   {
    flag=0
   led1.off();
   }
   delay(1000);
   
    }
   
 
   void GPS() // >>>>>> GPS function <<<<<
    {
      
    times=millis(); // Number of milliseconds passed since the program started.
    
    for(;;) // infinite loop to get location
    { 
    
   dur=millis()-times;
   if (dur>=5000) break; // if duration >= 5000 continue code.
   Serial.println("Searching for location..."); // detecting location... "on seial monitor"
   String altitude ;
  
  while (Serial2.available() > 0) // GPS connecting on Serial 2 of Arduino + if serial >1 = read location
   gps.encode(Serial2.read()); // read location from gps module
  
  if (gps.location.isUpdated())  // if location is updated "detected"
   {
    Serial.print("LAT="); Serial.print(gps.location.lat(), 6);      // print longitude "GPS ordinates in degrees with 6 decimal accuracy"
    Serial.print("  LNG="); Serial.println(gps.location.lng(), 6); // pting latitude   "GPS ordinates in degrees with 6 decimal accuracy"
    
   }
  
   }
   
     // >>>>> variables to widget  <<<<<
     
     int index = 0;              // index of location >>> Maps widget
     lat=gps.location.lat();    // save latitude to var "lat"
     lon=gps.location.lng();   //  save longitude to var "long"
  
     myMap.location(index, lat, lon, "value"); // send variables to widget on application
   }
  
void setup()
{
  // Debug console
  
  Serial.begin(9600);
   Serial2.begin(9600);
          
  //Serial.print("GPS start!!");
  
 EspSerial.println("AT+CWMODE=1"); // ESP2866 Web Server
  // Set ESP8266 baud rate
  EspSerial.begin(ESP8266_BAUD); 
  delay(10);
 myservo.attach(9); // servo signal pin
 
  Blynk.begin(auth, wifi, ssid, pass); // use variable data to connect blynk
  
pinMode(metal,INPUT); // get from metal detector


// >>>> send to drivers <<<<
pinMode(m1AR,OUTPUT);pinMode(m1BR,OUTPUT);
pinMode(m2AR,OUTPUT);pinMode(m2BR,OUTPUT);
pinMode(m1AL,OUTPUT);pinMode(m1BL,OUTPUT);
pinMode(m2BL,OUTPUT);pinMode(m2AL,OUTPUT);

 myservo.write(0); // intial postion for servo motor to insure the intial postion
 

  timer.setInterval(1000L, robot); // Setup a function to be called every second
}

void loop()
{
  Blynk.run(); // start blynk "blynk connected function "
  timer.run();  // start robot
}
