void recvWithEndMarker() {
int size = 0;
static byte ndx = 0;
char endMarker = '\n';
char rc;

// if (Serial.available() > 0) {
while (Serial.available() > 0 && newData == false) {
rc = Serial.read();

if (rc != endMarker) {
receivedChars[ndx] = rc;
ndx++;
if (ndx >= numChars) {
ndx = numChars - 1;
}
}
else {
receivedChars[ndx] = '\0';// terminate the string
ndx = 0;
newData = true;
}
}
}
