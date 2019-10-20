void set(){
mySerial.begin(9600);
mySerial.setTimeout(20);
pinMode(base_pin, OUTPUT);
pinMode(h1_pin, OUTPUT);
pinMode(h2_pin, OUTPUT);
pinMode(h3_pin, OUTPUT);
pinMode(mag, OUTPUT);
base.attach(base_pin);
h1.attach(h1_pin);
h2.attach(h2_pin);
h3.attach(h3_pin);
Serial.begin(9600);
Serial.setTimeout(20);
delay(1000);
Serial.println("Entering main loop");
servo_write(0, 0, 0, 0, 0);

}
