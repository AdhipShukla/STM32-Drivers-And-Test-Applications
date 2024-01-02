#include <SPI.h>

// Define ALTERNATE_PINS to use non-standard GPIO pins for SPI bus

#ifdef ALTERNATE_PINS
  #define HSPI_MISO   26
  #define HSPI_MOSI   27
  #define HSPI_SCLK   25
  #define HSPI_SS     32
#else
  #define HSPI_MISO   12
  #define HSPI_MOSI   13
  #define HSPI_SCLK   14
  #define HSPI_SS     15
#endif
#define LED_HIG       0x50
#define BUT_GET       0x51
#define LED_LOW       0x52
#define Dummy_Write    0x10

static uint8_t InHigh =(uint8_t)0;
static uint8_t Button_State = 0;
static uint8_t Dummy_Read = 0;
static const int spiClk = 500000; // 1 MHz
static int newreq    =  1;
//uninitalised pointers to SPI objects
SPIClass * hspi = NULL;
static boolean PrevSig = 0;
static boolean CurrSig = 0;
static int InterruptPin = 35;

void setup() {
  //initialise instance of the SPIClass attached to HSPI
  Serial.begin(115200);
  hspi = new SPIClass(HSPI);
  pinMode(InterruptPin,INPUT);
  //clock miso mosi ss
#ifndef ALTERNATE_PINS
  //initialise hspi with default pins
  //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  hspi->begin();
#else
  //alternatively route through GPIO pins
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); //SCLK, MISO, MOSI, SS
#endif

  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(HSPI_SS, OUTPUT); //HSPI SS
  digitalWrite(HSPI_SS, HIGH);
}
boolean DetectRisingEdge(){
  while(1){
    PrevSig = CurrSig;
    CurrSig = (boolean)digitalRead(InterruptPin);
    if(PrevSig != CurrSig){
      Serial.println();
      Serial.println("Curr Sig " + String(CurrSig)); 
    }
    if(CurrSig == 1){
      break;
    }
  }
  return 1;
}
// the loop function runs over and over again until power down or reset
void loop() {
  char cmd[8]={0};
  char c;
  uint8_t i=0;
  
  if(newreq==1){
    Serial.println();
    Serial.println("Enter Command");
    newreq=0;
  }
  if(Serial.available()>0){
    while(Serial.available()>0){
      c=Serial.read();
      if(c=='\n' || i==8){
        Serial.println("Command:" + String(cmd));
        break;
      }
      cmd[i]=c;
      i++;
    }
    if (!strcmp(cmd,"LED_HIG")){
      hspi_send_command(1);
    } 
    else  if(!strcmp(cmd,"BUT_GET")){
      hspi_send_command(2);
    }
    else if (!strcmp(cmd,"LED_LOW")){
      hspi_send_command(3);
    }
    else{
      Serial.println("Command not found: " + String(cmd));
    }
    newreq=1;
  }
  delay(1000);
  if(!(Serial.available()>0)){
    Serial.println();
    Serial.println("Auto Mode");
    hspi_send_command(2);
    if (InHigh){
      delay(100);
      hspi_send_command(1);
    } else {
      delay(100);
      hspi_send_command(3);
    }
  }
}

void hspi_send_command(uint8_t Cmd) {
 if (Cmd==1){
    Serial.println("Sending LED ON");
    byte data_on = LED_HIG; // data 1 to turn on LED of slave
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS, LOW);
    hspi->transfer(data_on);
    delay(10);
    digitalWrite(HSPI_SS, HIGH);
    hspi->endTransaction();
    DetectRisingEdge();
  }else if(Cmd==2){
    Serial.println("Sending Button Read");
    byte But_Sta = BUT_GET; // data 0 to turn off LED of slave
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS, LOW);
    Dummy_Read = hspi->transfer(But_Sta);
    delay(10);
    digitalWrite(HSPI_SS, HIGH);
    hspi->endTransaction();
    Serial.println("Dummy Read: " + String(Dummy_Read));
    DetectRisingEdge();
    delay(500);
    Serial.println("Sending Dummy Byte To Fetch Data");
    But_Sta = Dummy_Write;
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS, LOW);
    Button_State = hspi->transfer(But_Sta);
    digitalWrite(HSPI_SS, HIGH);
    hspi->endTransaction();
    DetectRisingEdge();
    Serial.println("Button State: " + String(Button_State));
    if (Button_State == 12){
      InHigh = (uint8_t)1;
    }else{
      InHigh = (uint8_t)0;
    }
  }
  else if(Cmd==3){
    Serial.println("Sending LED OFF");
    byte data_off = LED_LOW;
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS, LOW);
    hspi->transfer(data_off);
    delay(10);
    digitalWrite(HSPI_SS, HIGH);
    hspi->endTransaction();
    DetectRisingEdge();
  }
}