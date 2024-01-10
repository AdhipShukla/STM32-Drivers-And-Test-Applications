void setup() {
  Serial.begin(9600);
  
  // Define the LED pin as Output
  pinMode (13, OUTPUT);
  
  Serial.println("Arduino UART Receiver");
  Serial.println("-----------------------------");
    
}


void loop() {

  //digitalWrite(13, !digitalRead(13)); 
  //wait until something is received
  while(! Serial.available());
  //read the data
  char in_read=Serial.read();
  if(in_read =='\n')
    digitalWrite(13, !digitalRead(13));
  //print the data
  Serial.print(in_read);

}