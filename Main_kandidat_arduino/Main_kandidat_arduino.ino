#include <SoftwareSerial.h>
/*
   All the resources for this project: https://randomnerdtutorials.com/
   Modified by Andreas Jackson
   Created by FILIPEFLOP
*/
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#define SS_PIN 10
#define RST_PIN 9
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
#define N_TAGS 36
#define BUF_LEN 4
#define ERROR_MSG '+'
#define RIGHT '>'
#define LEFT '<'
#define STOP_BLINK 's'
#define LED_LEFT_PIN 7
#define LED_RIGHT_PIN 8

unsigned long previousMillis = 0;
unsigned long currentMillis = millis();
const long interval = 400;

uint8_t RFIDtagHEX[N_TAGS][4]={{0xC0,0x0E,0x3,0x0}, {0xC3,0x10,0x3,0x0}, {0xB8,0x0F,0x3,0x0}, {0x98,0x0F,0x3,0x0}, {0xA3,0x0E,0x3,0x0}, {0xC4,0x0F,0x4,0x0}, {0x0A,0x0E,0x4,0x1}, {0xFC,0x11,0x3,0x0}, {0xCE,0x0E,0x4,0x0}, {0xDF,0x0E,0x3,0x0}, {0xDC,0x11,0x3,0x0}, {0x19,0x11,0x3,0x1}, {0xE2,0x0F,0x4,0x0}, {0x9D,0x10,0x4,0x0}, {0xA5,0x0F,0x4,0x0}, {0xAF,0x0E,0x4,0x0}, {0xBF,0x11,0x3,0x0}, {0xFB,0x11,0x3,0x0}, {0xE4,0x10,0x3,0x0}, {0xC3,0x0F,0x4,0x0}, {0xEC,0x0E,0x4,0x0}, {0xC4,0x10,0x3,0x0}, {0x99,0x0F,0x3,0x0}, {0xBA,0x10,0x4,0x0}, {0x1A,0x11,0x3,0x1}, {0x87,0x0F,0x4,0x0}, {0xCD,0x0E,0x4,0x0}, {0xC1,0x0E,0x3,0x0}, {0x01,0x0F,0x4,0x1}, {0x86,0x0F,0x4,0x0}, {0x04,0x10,0x3,0x1}, {0xA4,0x0F,0x4,0x0}, {0xA5,0x10,0x3,0x0}, {0xFE,0x0E,0x3,0x0}, {0xA4,0x10,0x3,0x0}, {0x9C,0x10,0x4,0x0}};
String commonID[4] = {"04 ", " ", " ", "2 8B 5F 8"};
uint8_t IDnamn[36] = {11, 12, 13, 14, 21, 22, 23, 24, 25, 26, 27, 31, 32, 33, 34, 35, 36, 37, 41, 42, 43, 44, 45, 46, 51, 52, 53, 54, 55, 56, 61, 62, 63, 64, 65, 66};

char TxBuffer[BUF_LEN] = {0, 0, 0, '\n'};
char oldTxBuffer[BUF_LEN] = {0, 0, 0, '\n'};
int noCardCounter=0;
bool rightLedState = HIGH;
bool leftLedState = HIGH;
int blinkState = 0;
bool RFID_blink=false;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);
SoftwareSerial mySerial(4, 3);
//String resultString, resultString2;

void setup(void) {
  pinMode(LED_LEFT_PIN,OUTPUT);
  pinMode(LED_RIGHT_PIN,OUTPUT);
  digitalWrite(LED_LEFT_PIN,HIGH);
  digitalWrite(LED_RIGHT_PIN,HIGH);
  mySerial.begin(9600);
  //Serial.begin(9600);
  SPI.begin();    // Initiate  SPI bus
  mfrc522.PCD_Init();   // Initiate MFRC522
  if (tcs.begin()) {
 
   //Serial.println("Found sensor");
  
  // send_buffer();
  
  } else {
  //Serial.println("No TCS34725 found ... check your connections");
  while (1);
  }
  delay(100);
  for(int i=0;i<4;i++){
    rightLedState=!rightLedState;
    leftLedState=!leftLedState;
    digitalWrite(LED_LEFT_PIN,leftLedState);
    digitalWrite(LED_RIGHT_PIN,rightLedState);
    delay(80);
  }
}
/*
char colorOf(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
  if (c > 140)
  {
  return 'w';
  }
  else
  {
  return 'b';
  }
}
*/
void print_color_info(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
  Serial.print("R:");
  printWithNDigits(r,5,' ');
  Serial.print(", G:");
  printWithNDigits(g,5,' ');
  Serial.print(", B:");
  printWithNDigits(b,5,' ');
  Serial.print(", C:");
  printWithNDigits(c,5,' ');
  Serial.println("]");
}
void printWithNDigits(int d,int n, char space)
{
  for(int i=pow(10,n-1);i>0;i/=10)
  {
  if(d<i) Serial.print(space);
  }
  //Serial.print(d);
}
void send_buffer()
{
  //Serial.write(TxBuffer, BUF_LEN);
  mySerial.write(TxBuffer, BUF_LEN);
}
bool buffer_has_updated()
{
  for (int n = 0; n < 3; n++)
  {
  if (TxBuffer[n] != oldTxBuffer[n]) {
    return true;
  }
  }
  return false;
  //TxBuffer[BUF_LEN] != oldTxBuffer[BUF_LEN];
}
uint16_t oldC=0;
void loop(void) {
  uint16_t r, g, b, c, colorTemp, lux;
  char color;
  double colorData, colorMed;
  int colorDataSend;
  tcs.getRawData(&r, &g, &b, &c);
  
  uint16_t cMedel=(oldC+c)/2;
  oldC=c;
  
  //colorTemp = tcs.calculateColorTemperature(r, g, b);
  //colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  //lux = tcs.calculateLux(r, g, b);
  
  unsigned long currentMillis = millis();   
/*
  Serial.print('A');
  int tagNr =5;
  Serial.write('0'+tagNr);
  Serial.print("\tr:");
  Serial.print(r);
  Serial.print("\tg:");
  Serial.print(g);
  Serial.print("\tb:");
  Serial.print(b);
  Serial.println(" ");
  */
  colorMed = (r+g+b)/3.0;
  colorData = (g - colorMed)*16.5-10.0;
 
  if(colorData < 150.0)
  {
    colorDataSend = cMedel;
  }
  else 
  {
    colorDataSend = 255;
  }
  if(colorDataSend>255)
  {
    colorDataSend=255;
  }else if(colorDataSend<0){
    colorDataSend=0;
  }
  
  
 

  // Look for new RFID-tag
  if (  mfrc522.PICC_IsNewCardPresent())
  {
  // Select one of the RFID-tag
  if (  mfrc522.PICC_ReadCardSerial())
  {
    //Show UID on serial monitor
    //  Serial.print("UID tag :");
    String content = "";
    for (byte i = 0; i < mfrc522.uid.size; i++)
    {
      // Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
      // Serial.print(mfrc522.uid.uidByte[i], HEX);
      content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
      content.concat(String(mfrc522.uid.uidByte[i], HEX));
    }
    //Serial.println();
    //Serial.print("Message : ");
    content.toUpperCase();
    // Loopa array returna n+1  {{"CA CF A9 DF","A11"} alltså A11.
    for (int n = 0; n < N_TAGS; n++)
    {
      String resultString = "";
      String resultString2 = "";
      
      String hexfaker = String(RFIDtagHEX[n][0],HEX);
      hexfaker.toUpperCase();
      
      if(hexfaker.length()==1)
      {
        String nolla = "0";
        resultString = resultString2+commonID[0]+nolla+hexfaker;
        resultString2 = resultString;
      }else{
        resultString = resultString2+commonID[0]+hexfaker;
        resultString2 = resultString;
      }
      
      hexfaker = String(RFIDtagHEX[n][1],HEX);
      hexfaker.toUpperCase();
      if(hexfaker.length()==1)
      {
        String nolla = "0";
        resultString = resultString2+commonID[1]+nolla+hexfaker;
      }else{
        resultString = resultString2+commonID[1]+hexfaker;
      }
      resultString2 = resultString;
      hexfaker = String(RFIDtagHEX[n][2],HEX);
      
      hexfaker.toUpperCase();
      
      resultString = resultString2+commonID[2]+hexfaker;
      
      resultString2 = resultString;
      hexfaker = String(RFIDtagHEX[n][3],HEX);
      hexfaker.toUpperCase();
      resultString = resultString2+commonID[3]+hexfaker;
      resultString2 = resultString;
      //Serial.print(resultString+".");
      
      if (resultString == content.substring(1)) {
        TxBuffer[0] = IDnamn[n];
        // Serial.write(IDnamn[n]);
      }
    }
  }
  
  noCardCounter=0;
  }else{
  noCardCounter++;
  if(noCardCounter>=2){
    TxBuffer[0] = 0;
  
  }
  }
//  color = colorOf(r, g, b, c);
 
  // TxBuffer[1] = color;


 
  char incomingData;
  //Serial.print(incomingData);
// mySerial
  if (mySerial.available() > 0)
  {
    // mySerial
    incomingData = mySerial.read();
    if(incomingData == ERROR_MSG)
    {
      send_buffer();
      //Serial.print(incomingData);
    }
    else if(incomingData == '?')
    {
      TxBuffer[1] = colorDataSend;
      //send_buffer();
    }
    else if(incomingData == '>')
    {
      blinkState=1;
      //Serial.print(incomingData);
    }
    else if(incomingData == '<')
    {
      blinkState=2;
      //Serial.print(incomingData);
    }else if(incomingData == STOP_BLINK)
    {
      blinkState=0;
      //Serial.print(incomingData);
    }
  }
    if (buffer_has_updated()) {
      if(TxBuffer[0]>0)
      {
        RFID_blink=true;
        leftLedState=HIGH;
        rightLedState=HIGH;
        previousMillis=currentMillis;
      }
    send_buffer();
    oldTxBuffer[0] = TxBuffer[0];
    oldTxBuffer[1] = TxBuffer[1];
    oldTxBuffer[2] = TxBuffer[2];
    // Ta bort om ej vill ha spam
   // TxBuffer[0] = 0;
  }
 
  if(RFID_blink&&(currentMillis - previousMillis>= 300))
  {
    //if(currentMillis - previousMillis >= interval)
    leftLedState=LOW;
    rightLedState=LOW;
    RFID_blink=false;
  }else if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    
    if(blinkState==1)
    {
      if(rightLedState == LOW){
        rightLedState=HIGH;
      }else{
        rightLedState=LOW;
      }
      leftLedState=LOW;
    }else if(blinkState==2)
    {
      if(leftLedState == LOW){
        leftLedState=HIGH;
      }else{
        leftLedState=LOW;
      }
      rightLedState=LOW;
    }
    else
    {
      leftLedState = LOW;
      rightLedState = LOW;
    }
  }
  digitalWrite(LED_LEFT_PIN, !leftLedState);
  digitalWrite(LED_RIGHT_PIN, !rightLedState);
  
  //print_color_info(r,g,b,c);
  //Serial.print(colorData);Serial.print(", ");
  //printWithNDigits(colorDataSend,4,' ');
  //Serial.println(" ");
}
