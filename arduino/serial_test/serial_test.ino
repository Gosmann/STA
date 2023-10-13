htop// test serial communication between arduino and rpi

void setup() {
  // initialize both serial ports:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);     // /dev/ttyUSB0   on rpi
  Serial2.begin(9600);    // /dev/ttyS0     on rpi
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
  char buffer[50] = {0};
  
  sprintf(buffer, "Command number [%d] \n", i); 
  Serial.print(buffer);
  //Serial2.print(buffer);
  
  i++;
  
  //if(i%2 == 0) digitalWrite(LED_BUILTIN, HIGH);   
  //else digitalWrite(LED_BUILTIN, LOW);

  
  while ( Serial2.available() == 0 ) {
  
  }
  
    char c = Serial2.read();
    if( c == '1' ){
      digitalWrite(LED_BUILTIN, HIGH);   
      //break;
    }
    else if( c == '0' ){
      digitalWrite(LED_BUILTIN, LOW);   
    }
    //Serial1.write(inByte);
  

  //delay(1000);
  
  
}
