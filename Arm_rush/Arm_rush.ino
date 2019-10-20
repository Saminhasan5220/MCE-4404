#include <SoftwareSerial.h>
#include <Ramp.h>
#include<Servo.h>

bool bluetooh=false,usb = true;
SoftwareSerial mySerial(10, 11); // RX | TX
rampFloat Base,H1,H2,H3;
int time = 5000;


const byte numChars = 254;
char receivedChars[numChars];
String data;
boolean newData = false;
const int RX = 10, TX = 11, VCC = 10, GND = 12;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Servo base,h1,h2,h3;
int base_pin=6, h1_pin=5 , h2_pin=4, h3_pin=3, mag=13;
float a,b,c,d,e;
float last_a1,last_a2,last_a3,last_a4,last_a5; 
float base_offset=64,h1_offset=10,h2_offset=86,h3_offset=90;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{

set();
}
void loop() {
get(&a,&b,&c,&d,&e);
servo_write(a,b,c,d,e);
//Serial.println("Looped");
}
