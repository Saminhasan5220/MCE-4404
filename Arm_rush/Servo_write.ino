void servo_write(float a1,float a2,float a3,float a4,int a5){
 if(a5 != last_a5){
if(a5 ==1){
digitalWrite(mag,HIGH);
Serial.println("Magnet activated");  
}
else{
digitalWrite(mag,LOW);
Serial.println("Magnet deactivated");  
}
}

if(last_a1 != a1 || last_a2 != a2 || last_a3 != a3 || last_a4 != a4){
Base.go(a1, time, LINEAR, ONCEFORWARD);
H1.go(a2, time, LINEAR, ONCEFORWARD);
H2.go(a3, time, LINEAR, ONCEFORWARD);
H3.go(a4, time, LINEAR, ONCEFORWARD);
int Time = millis();
while(Base.value() != a1 || H1.value() != a2 || H2.value() != a3 || H3.value() != a4){
Serial.print("Base = ");
Serial.print(Base.value());
Serial.print("    ");

Serial.print("H1 = ");
Serial.print(H1.value());
Serial.print("    ");

Serial.print("H2 = ");
Serial.print(H2.value());
Serial.print("    ");

Serial.print("H3 = ");
Serial.print(H3.value());
Serial.println("    ");

base.write(Base.value()+base_offset);
h1.write(H1.value()+h1_offset);
h2.write(H2.value()+h2_offset);
h3.write(H3.value()+h3_offset);
Base.update();
H1.update();
H2.update();
H3.update();
//if(millis()> Time + time - 20000)
//break;
}
}
base.write(Base.value()+base_offset);
h1.write(H1.value()+h1_offset);
h2.write(H2.value()+h2_offset);
h3.write(H3.value()+h3_offset);
last_a1 = a1;
last_a2 = a2;
last_a3 = a3;
last_a4 = a4;
last_a5 = a5;
}
/*
base.write(a1+base_offset);
h1.write(a2+h1_offset);
h2.write(a3+h2_offset);
h3.write(a4+h3_offset);
mySerial.print('H1=');
mySerial.print(a1+base_offset);
mySerial.print(',');
mySerial.print('H2=');
mySerial.print(a2+h1_offset);
mySerial.print(',');
mySerial.print('H3=');
mySerial.print(a3+h2_offset);
mySerial.print(',');
mySerial.print('H4=');
mySerial.println(a4+h3_offset);

*/
