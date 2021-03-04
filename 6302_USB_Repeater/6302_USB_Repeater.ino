// Repeat anything read in

#define T2Tserial Serial1
#define baudRate 1000000
void setup() {
  Serial.begin(baudRate);
  T2Tserial.begin(baudRate);
}

void loop() {
  char inByte;
        
  if (Serial.available() > 0) {
    inByte = Serial.read();
    T2Tserial.write(inByte);
  }

  if (T2Tserial.available() > 0) {
    inByte = T2Tserial.read();
    Serial.write(inByte);
  } 

  /* Serial.println("Hello Peter!"); */
}
