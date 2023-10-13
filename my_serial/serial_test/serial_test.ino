// test serial communication between arduino and rpi

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);

}

int i = 0;

void loop() {
  /*
  // read from port 1, send to port 0:
  if (Serial1.available()) {
    int inByte = Serial1.read();
    Serial.write(inByte);
  }

  // read from port 0, send to port 1:
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial1.write(inByte);
  }
  */
  char buffer[50];
  sprintf(buffer, "Command number [%d]", i); 
  Serial.println(buffer);
  //Serial1.println(buffer);
  //Serial2.println(buffer);
  Serial3.println(buffer);
  i++;

  
  
  delay(1000);
  
}
