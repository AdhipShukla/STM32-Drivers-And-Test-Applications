
char Mes[]="ESP32 is Sending\n";

void setup() {
  Serial.begin(9600);
  
  // Define the LED pin as Output
  pinMode (13, OUTPUT);
  
 // Serial.println("Arduino Case Converter program running");
 // Serial.println("-------------------------------------");
  
    
}

char changeCase(char ch)
{
  if (ch >= 'A' && ch <= 'Z')
  ch = ch + 32;
    else if (ch >= 'a' && ch <= 'z')
  ch = ch - 32;  

  return ch;
}
void loop() {
  //digitalWrite(13, LOW); 
  //wait until something is received
  while(! Serial.available());
  //digitalWrite(13, HIGH); 
  //read the data
  char in_read=Serial.read();
  if(in_read =='\n')
    digitalWrite(13, !digitalRead(13));
  //print the data
  //Serial.write(changeCase(in_read));
  Serial.print(changeCase(in_read));
  
  /*for(int i=0; i<17; i++){
    Serial.write(Mes[i]);
  }
  delay(1000);*/
}