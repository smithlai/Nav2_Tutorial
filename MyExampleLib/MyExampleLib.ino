/*
  String to Integer conversion

  Reads a serial input string until it sees a newline, then converts the string
  to a number if the characters are digits.

  The circuit:
  - No external components needed.

  created 29 Nov 2010
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/StringToInt
*/

String inString = "";    // string to hold input
int nBlinkCnt = 0;
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // send an intro:
  Serial.println("\n\nString toInt():");
  Serial.println();
}

void loop() {
  readinput();
  if (nBlinkCnt>0){
    Serial.print("BlinkCnt:");
    Serial.println(nBlinkCnt);
    myblink();
    nBlinkCnt--;
  }
}
void readinput() {
  // Read serial input:
  while (Serial.available() > 0) {
    Serial.print("AAAAAAA: ");
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      int _cnt=inString.toInt();
      Serial.print("Get Value:");
      Serial.println(_cnt);
//      Serial.print("String: ");
//      Serial.println(inString);
      // clear the string for new input:
      inString = "";
      nBlinkCnt += _cnt;
    }
  }
}
// the loop function runs over and over again forever
void myblink() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(500);                       // wait for a second
}
