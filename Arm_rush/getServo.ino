void getServo(int *p)
{
recvWithEndMarker();
showNewData();
String P = getValue(receivedChars, ',', 0);
*p = P.toInt();

//mySerial.print("P:=");
//mySerial.println(*p);

}
