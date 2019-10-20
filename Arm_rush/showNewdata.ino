void showNewData() {
if (newData == true ) {
data = receivedChars;
Serial.print("Recieved data:");
Serial.println(receivedChars);
}
}

