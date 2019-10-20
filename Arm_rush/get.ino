void get(float *p, float *q, float *r, float *s, float *t)
{
recvWithEndMarker();
showNewData();
if(newData){
String P = getValue(receivedChars, ',', 0);
*p = P.toFloat();
String Q = getValue(receivedChars, ',', 1);
*q = Q.toFloat();
String R = getValue(receivedChars, ',', 2);
*r = R.toFloat();
String S = getValue(receivedChars, ',', 3);
*s = S.toFloat();
String T = getValue(receivedChars, ',', 4);
*t = T.toFloat();

Serial.print("Processed data:");
Serial.print(*p);
Serial.print(',');
Serial.print(*q);
Serial.print(',');
Serial.print(*r);
Serial.print(',');
Serial.print(*s);
Serial.print(',');
Serial.println(*t);
newData = false;
}
//mySerial.print('T');//mySerial.println(*t);
/*
mySerial.print("P,q,r,s=");
mySerial.print(*p);
mySerial.print(',');
mySerial.print(*q);
mySerial.print(',');
mySerial.print(*r);
mySerial.print(',');
mySerial.println(*s);*/
}
