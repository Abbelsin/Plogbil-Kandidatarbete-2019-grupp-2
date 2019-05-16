/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c

  * @brief          : Main program body
  * @author			: Albin Lejåker
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include "DistSensor.h"
#include "varvtal.h"
#include "motorstyrning.h"
#include <string.h>
#include "tcs3472.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// buffer size anger antal bytes som skickas/tas emot i varje meddelande
#define TX_BUF_SIZE 16
#define RX_BUF_SIZE 8
#define RX2_BUF_SIZE 4
#define TX2_BUF_SIZE 4

#define FORWARD_TIME 2000
#define START_DELAY 1000

//driveState defines
#define LINE_FOLLOW 4
#define BLIND_FORWARD 5
#define BLIND_LEFT_ONE 3
#define BLIND_LEFT_TWO 10
#define WAIT_FOR_ORDER 6
#define U_TURN 9

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char TXbuffer[TX_BUF_SIZE] = "Lost memory    \n";
char tempTXbuffer[TX_BUF_SIZE];
char RXbuffer[RX_BUF_SIZE];
char RX2buffer[RX2_BUF_SIZE];
char TX2buffer[TX2_BUF_SIZE];

uint8_t byte;
uint8_t RX_error=0; //lsb=uart6, bit 2 = uart1
uint8_t connected=0;
uint8_t newMessage = 0;
uint16_t buttonCounter = 0;
uint32_t prevMillis=0;
uint32_t prevPIDMillis=0;
uint32_t debounceInterval = 100;
uint32_t driveState=0;//avgör vilket läge Fordonet befinner sig i
uint8_t stateChanged=0;
uint32_t timeFlag1=0;
uint32_t timeFlag2=0;
uint32_t timeFlag3=0;
char lastColor='b';
uint8_t waiting=0;
uint8_t newOrder=0;
uint8_t globalRoadType=0;
uint8_t lastRoadType=0;
uint8_t lastRoadIndex=0;
uint8_t lastRFIDindex=0;

uint8_t errorInt=0;
#define E_CANNOT_OBEY 0x01
#define E_BT_NOT_AVAILABLE 0x02
#define E_ARDUINO_LOST 0x04

uint8_t tryAgainConuter=0;
uint8_t clearOrder = 0;
uint8_t colorLimit = 145;//används för schmitt trigger i IR sensprer
char lastReadColor='b';
uint8_t printDebugInfo=1;//printar massor info i BT
int subStage=1;//används för att räkna delsteg i en manöver
int8_t R_speed=100;//andvänds för att reglera rakkörning
int8_t L_speed=100;
uint8_t continueRutt=0;

uint8_t byteOfBools=0; //test för att komprimera bool-flaggor
#define BOOL1 byteOfBools&0x01
#define BOOL2 byteOfBools&0x02
#define BOOL3 byteOfBools&0x04
#define BOOL4 byteOfBools&0x08
#define BOOL5 byteOfBools&0x10
#define BOOL6 byteOfBools&0x12
#define BOOL7 byteOfBools&0x14
#define BOOL8 byteOfBools&0x18

#define BOOL1_TRUE byteOfBools|=0x01
#define BOOL1_FALSE byteOfBools&=~0x01
#define BOOL3_TRUE byteOfBools|=0x04
#define BOOL3_FALSE byteOfBools&=~0x04
#define BOOL5_TRUE byteOfBools|=0x10
#define BOOL5_FALSE byteOfBools&=~0x10
uint8_t obeyRFID = 1; //avgör om roboten ska reagera på RFID taggar

//PID variabler
#define MID_POINT 150
double integral=0.0;
double derivative=0.0;
int errorPrior=0;
int meanErrorPrior=0;
//för raksträcka:

const double KP1 = 0.08; //0.08
const double KI1 = 3.73e-13;
const double KD1 = 6.87e+10;

double KP = 0.2813;
double KI = 0.0001364;
double KD = 7.58e+10;
double KE = 0.0213;

const double KP_forward = 0.08;
const double KI_forward = 4.78e-13;
const double KD_forward = 6.87e+10;
const double KE_forward = 0.02;

const double KP_Right = 0.18876;
const double KI_Right = 4.35e-13;
const double KD_Right = 9.14397e+10;
const double KE_Right = 0.02662;

const double KP_Left = 0.1085;
const double KI_Left = 7.7e-05;
const double KD_Left = 6.87e+10;
const double KE_Left = 0.02662;

/*
double KP = 0.1;
double KI = 3.7e-13;
double KD = 6.87e+10;
*/

int PIDoutput=0;
int Doutput=0;
double maxPID=1000000.0;
double minPID=-1000000.0;
double* adjustThis; //pekare för justering av parametrar
int memError=0; //extra error variabel som ökar utanför linjen

/* USER CODE END PV */

/* Private function prototypes (genererade funktioner)------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//void measureDistance();
//generella funktioner

void stateFunction(void);
void send_RFID_info(uint8_t RFIDindex);
void RFID_decision(uint8_t RFIDindex);

//uart funktioner
void clearRXbuffer(UART_HandleTypeDef *huart);
void copyToTXbuffer(char* a);
void BT_println(char* s);
void BT_printdouble(double d);
void BT_printint(int a);
void BT_repeat();
void BT_wtf();
void KTS_message_recieved(UART_HandleTypeDef *huart);
void Atmel_message_recieved(UART_HandleTypeDef *huart);
void Atmel_println(char* s);
void Atmel_printchar(char* s);
void double_to_char(double f, char* buffer);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

//Linjeföljningsfunktioner
char binaryColor(uint8_t val);
void lineFollow1();
void lineFollowForward();
void lineFollowForwardOld();
void lineFollowLeft();
void lineFollowReglering();
void lineFollowRegleringLeft();
void lineFollowRegleringRight();
void lineFollowRegleringForward();
void lineFollowRegleringForward2();
void regulateStanding();
void steer(int amount, int midlevel, int minimumlevel);
void steer2(int amount);
void steer3(int amount, int midlevel, int minimumlevel, int ajustL);
void steer4(int amount, int midlevel, int minimumlevel, int maxlevel, int ajustL);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART6_UART_Init();
	MX_I2C3_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();

	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim4);

	//Kod för färgsensorer, användes ej i stm32 i slutgiltig produkt eftersom ATmega sköter det
	/*TCS3472 colorSensors;
	TCS3472_Color colors;
	//TCS3472_readColor(&colorSensors,&colors);
	TCS3472_setup(&colorSensors,&hi2c3,0x29);
	uint16_t data;
	//_TCS3472_read16bitReg(&colorSensors, 0x16, &data);*/

	//struct för Time of flight sensorer
	//Struct with address of VL53L0X and clockspeed of I2C.
	VL53L0X_Dev_t x[3] = {
		[0] = {
			.I2cDevAddr = 0x52,//VL53L0X adress enligt databladet.
			.comms_speed_khz = 100//Enligt Cube MX så ska det vara 100 KHz
		},
		[1] = {
			.I2cDevAddr = 0x52,//VL53L0X adress enligt databladet.
			.comms_speed_khz = 100//Enligt Cube MX så ska det vara 100 KHz
		},
		[2] = {
			.I2cDevAddr = 0x52,//VL53L0X adress enligt databladet.
			.comms_speed_khz = 100//Enligt Cube MX så ska det vara 100 KHz
		}
	};
	//Kolla ifall data är redo
	uint8_t DataReady = 0;
	//Avståndsdata sparas i
	VL53L0X_RangingMeasurementData_t ToF_RangingData[3] = {0};

	//Stäng av alla pinnar
	//GPIOC har pinne 8 och 5, A har 12.
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	//Dra igång en pin i taget och ange address
	//Använd inte LSB, den används för annat.
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	init_GPIO_Hezt(&x[0], 0x02);
	x[0].I2cDevAddr = 0x02;
/*
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	init_GPIO_Hezt(&x[1], 0x04);
	x[1].I2cDevAddr = 0x04;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	init_GPIO_Hezt(&x[2], 0x06);
	x[2].I2cDevAddr = 0x06;*/


	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	__HAL_UART_ENABLE_IT(&huart6, UART_IT_TC);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);


	//vänta på svar från dator
	char re[2]={0,0};
	char expected[2] = {'*','\n'};
	while(re[0]!='*'){//wait for the computer to connect
		HAL_UART_Transmit_IT(&huart6,&expected,2);
		HAL_UART_Receive_IT(&huart6,&re,2);
		RXbuffer[0]=re[0];
		RXbuffer[1]=re[1];
		HAL_Delay(400);
	}
	HAL_UART_AbortReceive_IT(&huart6);


	//Startmeddelande
  	char initMessage[TX_BUF_SIZE];
  	int i = 0;
  	initMessage[i++]='i';
  	initMessage[i++]=(uint8_t)RX_BUF_SIZE;
  	initMessage[i++]=(uint8_t)TX_BUF_SIZE;
  	while(i<TX_BUF_SIZE-1)
  	{
  		initMessage[i++]=' ';
  	}
  	initMessage[TX_BUF_SIZE-1]='\n';

  	copyToTXbuffer(initMessage);
  	HAL_UART_Transmit_IT(&huart6, (uint8_t *)TXbuffer,TX_BUF_SIZE);

  	while(!connected){
  		timeFlag1 = millis();
  	}

  	//enable interrupt för uart mellan arduino och stm
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	Atmel_printchar("?");//skicka request för sensordata till ATmega

	HAL_UART_Receive_IT(&huart6, (uint8_t *)RXbuffer, RX_BUF_SIZE);  //läs UART från Bluetooth
	HAL_UART_Receive_IT(&huart1, (uint8_t *)RX2buffer, RX2_BUF_SIZE);  //läs UART från Atmega
	/*if((RX_error)&((uint8_t)1)){
		clearRXbuffer(&huart6);
		RX_error&=~1;
	}
	if(RX_error&2){
		clearRXbuffer(&huart1);
		RX_error&=~2;
	}*/

	//läs avståndssensorer (bara en avståndssensor används för tillfället)
	for(int j = 0; j<1; j++)
	{
		VL53L0X_GetMeasurementDataReady(&x[j], &DataReady);
		//for(int i = 0; i< 3;)
			//{
			//VL53L0X_GetMeasurementDataReady(&x[j], &DataReady);
		if(DataReady == 1)
		{
			//ToF_RangingData ger avståndsvärdet.
			VL53L0X_GetRangingMeasurementData(&x[j], &ToF_RangingData[j]);
			//Clear biten i sensorn som DataReady kopierar
			VL53L0X_ClearInterruptMask(&x[j], VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
		}
	}
	int range = ToF_RangingData[0].RangeMilliMeter;

	/*if(range<100&&driveState!=0){
		driveState=0;//emergency stop
		stateChanged=1;
		BT_println("E5");
	}*/

	stateFunction(); //funktion som bestämmer vad som ska exekeveras loopas

	if(timeFlag3-millis()>600&&BOOL5){ //Om justering sker på P,I, eller D så printas det nya värdet
		BT_printdouble(*adjustThis);
		BOOL5_FALSE;
	}
	if((errorInt& E_CANNOT_OBEY)&&huart6.gState == HAL_UART_STATE_READY){
		BT_println("E4");
		errorInt&=~E_CANNOT_OBEY;
	}
	if(errorInt& E_ARDUINO_LOST){
		HAL_Delay(200);
		BT_println("E6");
		errorInt&=~E_ARDUINO_LOST;
	}
  }//end of while(1)
  /* USER CODE END 3 */
}
void stateFunction()
{
	if(stateChanged)
	{
	  switch(driveState)
	  {
	  case 0:
		  Atmel_printchar("s");
		  driveforward(0);
		  break;
	  case 1://drive forward test
		  Atmel_printchar("s");
		  reset_counter_R();
		  driveforward(100);
		  break;
	  case 2:
	  case BLIND_LEFT_ONE:
		  subStage=1;
		  timeFlag1=millis();
		  reset_counter_R();
		  reset_counter_L();
		  break;
	  case 4:
		  timeFlag1=millis();
		  reset_counter_R();
		  reset_counter_L();
		  break;
	  case BLIND_FORWARD:
		  Atmel_printchar("s");
		  timeFlag1=millis();
		  timeFlag2=timeFlag1+FORWARD_TIME;
		  reset_counter_R();
		  reset_counter_L();
		  subStage=1;
		  break;
	  case WAIT_FOR_ORDER:
		  Atmel_printchar("s");
		  driveforward(0);
		  reset_counter_R();
		  reset_counter_L();
		  break;
	  case 7: //justera efter kant
		  if(binaryColor(RX2buffer[1])=='w')
		  { //start turning left
			  timeFlag1=millis()+700;//when to stop turning left
		  }else
		  {
			  timeFlag1=millis()+300;
		  }
		  timeFlag2=millis();
		  timeFlag3=millis();

		  break;
	  case 8://remote
		  timeFlag1=millis();
		  reset_counter_R();
		  reset_counter_L();
		  if(RXbuffer[1]=='<'){
			  Atmel_printchar("<");
		  }else if(RXbuffer[1]=='>'){Atmel_printchar(">");
		  }else if(RXbuffer[1]=='^'){Atmel_printchar("s");
		  }
		  break;
	  case U_TURN:
		  timeFlag1=millis();
		  break;
	  case BLIND_LEFT_TWO:
		  timeFlag1=millis();
		  reset_counter_R();
		  reset_counter_L();
		  subStage=1;
		  break;
	  case 11: //left turn
		  timeFlag1=millis();
		  reset_counter_R();
		  reset_counter_L();
		  break;
	  }
	  stateChanged=0;
	}
	switch(driveState) //loopar kontinuerligt för den aktuella driveState
	{
	case 0:

	  break;
	case 1:

		if(read_counter_R()>8)
		{
		  driveforward(0);
		  char status[]={'R','0','L','0'};
		  status[1]=read_counter_R()+'0';
		  status[3]=read_counter_L()+'0';
		  BT_println(status);
		  reset_counter_R();
		}
		break;
	case 2:

		if(millis()-timeFlag1<100)
		{
			driveforward(millis()-timeFlag1);
		}else if(millis()-timeFlag1<100+100)
		{
			driveforward(100);

			//BT_println("one");
		}else if(millis()-timeFlag1<100+100+900)
		{
			hardcode_right();
			//turn_left(100);
			//BT_println("two");
		}else if(millis()-timeFlag1<100+100+900+400)
		{
			driveforward(100);
			//BT_println("three");
		}else
		{
			driveforward(0);
			//BT_println("four");
		}
		break;
	case BLIND_LEFT_ONE://linjef, sedan sväng

		//int sumDist = read_dist_R() + read_dist_L();
		switch(subStage){
		case 1:
			if(read_dist_R() + read_dist_L()<310)
			{
				//accelerera
				//driveforward((millis()-timeFlag1)>>1);
				lineFollowForward();
			}else if(read_dist_R() + read_dist_L()<310+560)
			{
				//driveforward(100);
				motor_R(100);
				motor_L(40);
			}else if(binaryColor(RX2buffer[1])=='b') //read_dist_R() + read_dist_L()<310+560||
			{
				subStage=2;
				driveforward(0);
			}
			break;
		case 2:
			motor_R(40);
			motor_L(40);
			if(binaryColor(RX2buffer[1])=='w')
			{
				motor_R(0);
				motor_L(0);
				subStage=3;
			}
			break;
		case 3:
			motor_L(-40);
			motor_R(0);
			if(binaryColor(RX2buffer[1])=='b')
			{
				subStage=4;
			}
			break;
		case 4:
			lineFollowReglering();
			break;
		}
		break;

	case LINE_FOLLOW://linjeföljning
		if(RXbuffer[2]=='f'){
			lineFollowRegleringForward2();
		}else if(RXbuffer[2]=='o'){
			lineFollowReglering();
		}else{
			if(globalRoadType==1||globalRoadType==6)//raksträcka
			{
				lineFollowRegleringForward();
				//lineFollowForward();
			}else if(globalRoadType==2)
			{
				lineFollowRegleringLeft();
			}else if(globalRoadType==3||globalRoadType==4||globalRoadType==5||globalRoadType==7){
				lineFollowRegleringRight();
			}else{
				lineFollowReglering();
				/*//Gammal kod för linjeföljing som inte använder PID-reglering
				if(binaryColor(RX2buffer[1])=='w')
				{
					if(lastColor!='w')
					{
						timeFlag1=millis();
					}
					if(millis()-timeFlag1>700)
					{  //minska ner ytterligare om det tar lång tid
						motor_L(-20);
						motor_R(40);
					}else{
						motor_L(25);
						motor_R(45);
					}
					lastColor='w';
				}else if(binaryColor(RX2buffer[1])=='b')
				{
					if(lastColor!='b')
					{
						timeFlag2=millis();
					}
					if(millis()-timeFlag1>800)
					{  //minska ner ytterligare om det tar lång tid
						motor_R(10);
						motor_L(40);
					}else{
						motor_R(25);
						motor_L(45);
					}

					lastColor='b';
				}*/
			}
		}

		break;
	case BLIND_FORWARD://rakt i korsning
		switch(subStage){
		case 1:

			if(millis()-timeFlag1<200)
			{
				//accelerera
				driveforward((millis()-timeFlag1)/2);
			}else if(read_dist_R() + read_dist_L()<500)
			{
				//reglerad körning framåt
				if(read_dist_R()<read_dist_L()-48)
				{
					if(R_speed<100)
						R_speed++;
					else
						L_speed--;
				}else if(read_dist_L()<read_dist_R()-48){
					if(L_speed<100)
						L_speed++;
					else
						R_speed--;
				}else{
					R_speed=100;
					L_speed=100;
				}
				motor_R(R_speed);
				motor_L(L_speed);
			}else
			{
				motor_L(45);
				motor_R(40);
				if(RX2buffer[1]>200){
					subStage=2;
					driveforward(0);
				}
			}
			break;
		case 2:
			if(RX2buffer[1]>=220){
				motor_R(40);
				motor_L(-40);
			}else{
				motor_R(0);
				motor_L(0);
				subStage=3;
			}
			break;
		case 3:
			if(RX2buffer[1]<220)
			{
				motor_L(40);
				motor_R(0);
			}else{
				subStage=4;
			}
			break;
		case 4:
			lineFollowReglering();
			break;
		}
		break;
	case WAIT_FOR_ORDER: //vänta på order
		driveforward(0);
		if(newOrder){
			if(newOrder=='<'){
				if(globalRoadType==5){
					driveState=BLIND_LEFT_TWO;
					stateChanged=1;
					Atmel_printchar("<");
				}else if(globalRoadType==6){
					driveState=BLIND_LEFT_ONE;
					stateChanged=1;
					Atmel_printchar("<");
				}else if(globalRoadType==7){
					driveState=BLIND_LEFT_TWO;
					stateChanged=1;
					Atmel_printchar("<");
				}else{
					errorInt|=E_CANNOT_OBEY;//kan inte köra dit
				}
			}else if(newOrder=='>'){
				if(globalRoadType==4||globalRoadType==5||globalRoadType==7){
					driveState=LINE_FOLLOW;
					stateChanged=1;
					Atmel_printchar(">");
				}else{
					errorInt|=E_CANNOT_OBEY;//kan inte köra dit
				}
			}else if(newOrder=='^'){
				if(globalRoadType==4||globalRoadType==7){
					driveState=BLIND_FORWARD;
					stateChanged=1;
				}else if(globalRoadType==6){
					driveState=LINE_FOLLOW;
					stateChanged=1;
				}else{
					errorInt|=E_CANNOT_OBEY;//kan inte köra dit
				}
			}else if(newOrder=='u'){
				if(globalRoadType>=4){
					driveState=U_TURN;
					stateChanged=1;
					Atmel_printchar("<");
				}else{
					errorInt|=E_CANNOT_OBEY;//kan inte köra dit
				}
			}
			newOrder=0;
		}
		break;
	case 7: //justera rotation efter vägkant
		/*if(binaryColor(RX2buffer[1])=='w'){
			motor_L(-30);//motor_R(32);
			if(lastColor=='b'){
				timeFlag2=millis();
				lastColor='w';
			}
		}else{
			motor_L(30);//motor_R(-32);
			if(lastColor=='w'){
				timeFlag3=millis();
				lastColor='b';
			}
		}
		if((timeFlag3-timeFlag2)<300 && (timeFlag2>timeFlag3)){
			motor_R(0);
			motor_L(0);
			lastColor='w';
			driveState=6; //ready to wait/drive according to command
			stateChanged=1;
		}*/
		if(millis()<timeFlag1)
		{
			motor_L(-60);//motor_R(40);
			if(binaryColor(RX2buffer[1])=='w'){
				timeFlag1=millis()+100;
			}
		}else if(millis()<timeFlag1+100){motor_L(0);
		}else{//sväng höger
			int timePassed= millis()-timeFlag1;
			const int minRSpeed = 33;
			int speed=50-(timePassed/20);
			if(speed<minRSpeed) speed=minRSpeed;

			motor_L(speed);motor_R(10);

			if(binaryColor(RX2buffer[1])=='w'){
				if(lastColor=='b'){
					motor_R(0);
					motor_L(0);
					driveState=6; //ready to wait/drive according to command
					stateChanged=1;
				}else{
					//going right in white=not good
					timeFlag1=millis()+200;
				}
				lastColor='w';
			}else{
				lastColor='b';
			}
		}
		break;
	case 8: //remote controll
		if(RXbuffer[1]=='<'){
				motor_R(40);
				motor_L(-40);
				obeyRFID=0;
		}else if(RXbuffer[1]=='>'){
			//turn_right(100);
				motor_R(-40);
				motor_L(40);
				obeyRFID=0;
			/* debug och mättester
			if(millis()-timeFlag1>=250)
			{//dont spam
				timeFlag1=millis();
				int tempSpeed = 10*(RXbuffer[2]-'0'+1);
				driveforward(tempSpeed);
				uint32_t distCM = read_dist_L()/10;
				BT_printint(distCM);
				if(distCM<30){
					turn_right(30);
				}else{
					driveforward(0);
				}
			}*/
		}else if(RXbuffer[1]=='^'){
			/*if(millis()-timeFlag1>=250){//dont spam
				timeFlag1=millis();
				int tempSpeed = 10*(RXbuffer[2]-'0'+1);
				driveforward(tempSpeed);
				int speedReading=read_speed_L(8);
				if(speedReading>0)
				{
					BT_printint(speedReading);
				}else{
					//BT_println("0");
				}

			}*/
			motor_R(50);
			motor_L(50);
			obeyRFID=0;
		}else if(RXbuffer[1]=='v'){
			motor_R(-50);
			motor_L(-50);
			obeyRFID=0;
		}else if(RXbuffer[1]=='a'){motor_R(-100);motor_L(100);
		}else if(RXbuffer[1]=='b'){motor_R(100);motor_L(-100);
		}else if(RXbuffer[1]=='c'){
			stateChanged=1;
			driveState=1;
		}else if(RXbuffer[1]=='d'){//right
			stateChanged=1;
			driveState=2;
		}else if(RXbuffer[1]=='e'){//left
			stateChanged=1;
			driveState=3;
		}else if(RXbuffer[1]=='f'){//line following
			if(RX2buffer[0]>0){
				continueRutt=1;
				RFID_decision(RX2buffer[0]);
			}else{
				stateChanged=1;
				driveState=LINE_FOLLOW;
			}
			obeyRFID=1;
		}else if(RXbuffer[1]=='g'){
			stateChanged=1;
			driveState=5;
		}else if(RXbuffer[1]=='h'){
			hardcode_left();
		}else if(RXbuffer[1]=='i'){
			stateChanged=1;
			driveState=7;//justera efter kant
		}else if(RXbuffer[1]=='s'){
			int tempSteer = 10*(RXbuffer[2]-'0')-50;
			steer(tempSteer,50,-50);
		}else if(RXbuffer[1]=='j'){
			if(RXbuffer[2]=='f')
				lineFollowRegleringForward();
			else
				lineFollowReglering();
			obeyRFID=0;
		}else if(RXbuffer[1]=='k'){
			regulateStanding();
			obeyRFID=0;
		}else{
			driveforward(0);
		}
		break;
	case U_TURN: // U-turn
		if(millis()-timeFlag1<100)
		{
			driveforward(millis()-timeFlag1);
		}else if(millis()-timeFlag1<100+200)
		{
			driveforward(100);
		}else if(millis()-timeFlag1<100+200+800)
		{
			hardcode_u_turn();

		}else if(millis()-timeFlag1<100+200+800+180)
		{
			driveforward(60);

		}else if(millis()-timeFlag1<100+200+800+180+700 && binaryColor(RX2buffer[1])=='b')
		{
			hardcode_u_turn();
		}else{
			lineFollowReglering();
		}
		break;
	case BLIND_LEFT_TWO: //sväng, sedan linjeföljning
		switch(subStage){
		case 1:
			if(millis()-timeFlag1<400)
			{
				//accelerera
				driveforward((millis()-timeFlag1)/4);
				//lineFollow1();
			}else if(read_dist_R() + read_dist_L()<440)
			{
				motor_R(100);
				motor_L(60);
			}else
			{
				motor_R(80);
				motor_L(48);
				if(RX2buffer[1]>200){
					subStage=2;
					driveforward(0);
				}
			}/*else if(millis()-timeFlag1<200+1000+400 && binaryColor(RX2buffer[1])=='b')
			{
				motor_R(100);
				motor_L(60);
			}else
			{
				driveforward(0);
				driveState = LINE_FOLLOW;
				stateChanged = 1;
			}*/
			break;
		case 2:
			motor_R(30);
			motor_L(-40);
			if(binaryColor(RX2buffer[1])=='b'){
				subStage=3;
				reset_counter_R();
				reset_counter_L();
			}
			break;
		case 3:
			if(read_dist_R() + read_dist_L()<200)
				lineFollowReglering();
			else
				lineFollowRegleringForward2();
			break;
		}
		break;
	case 11: //left turn (line follow)
		if(read_dist_R()<170){
			lineFollowRegleringForward2();
		}else if(read_dist_R()<485)
		{
			//driveforward(0);
			//lineFollowRegleringLeft();
			lineFollowReglering();
		}
		else{
			BOOL1_TRUE;
			lineFollowRegleringForward2();
		}

		break;
	}
}
void lineFollow1(){
	if(binaryColor(RX2buffer[1])=='w')
	{
		if(lastColor!='w')
		{
			timeFlag1=millis();
		}
		if(millis()-timeFlag1>1000)
		{  //minska ner ytterligare om det tar lång tid
			motor_L(-10);
			motor_R(40);
		}else{
			motor_L(30);
			motor_R(50);
		}
		lastColor='w';
	}else if(binaryColor(RX2buffer[1])=='b')
	{
		if(lastColor!='b')
		{
			timeFlag1=millis();
		}
		if(millis()-timeFlag1>1000)
		{  //minska ner ytterligare om det tar lång tid
			motor_R(10);
			motor_L(40);
		}else{
			motor_R(30);
			motor_L(50);
		}

		lastColor='b';
	}
}
void lineFollowForwardOld(){
	if(binaryColor(RX2buffer[1])=='w')
	{
		if(lastColor!='w')
		{
			timeFlag1=millis();
		}
		if(millis()-timeFlag2>2000){
			motor_L(25);
			motor_R(65);
		}else{
			motor_L(40);
			motor_R(60);//60
		}
		lastColor='w';
	}else if(binaryColor(RX2buffer[1])=='b')
	{
		if(lastColor!='b')
		{
			timeFlag2=millis();
		}
		if(millis()-timeFlag1>2000){
			motor_L(45);
			motor_R(25);
		}else{
			motor_L(60);
			motor_R(40);
		}
		lastColor='b';
	}
}
void lineFollowLeft(){
	if(binaryColor(RX2buffer[1])=='b')
		{
			if(lastColor!='b')
			{
				timeFlag2=millis();
			}
				if(millis()-timeFlag1>2500)
				{  //minska ner ytterligare om det tar lång tid
					motor_L(40);
					motor_R(45);
				}else{
					motor_L(35);
					motor_R(50);
				}

			lastColor='b';
		}else if(binaryColor(RX2buffer[1])=='w')
		{
			if(lastColor!='w')
			{
				timeFlag1=millis();
			}

				if(millis()-timeFlag1>1000)
				{  //minska ner ytterligare om det tar lång tid
					motor_L(0);
					motor_R(40);
				}else{
					motor_L(-15);
					motor_R(45);
				}

			lastColor='w';
		}
}
#define NOISE_TOLERANCE 8

void lineFollowRegleringLeft()
{
	uint32_t currentMillis=millis();
	//uint32_t iterationTime = currentMillis-prevPIDMillis;
	int pos;
	if(RX2buffer[3]=='\n'){
		pos = RX2buffer[1];
	}else{
		tryAgainConuter++;
		if(tryAgainConuter>=100){
			tryAgainConuter=0;
			driveState=0;
			stateChanged=1;
			errorInt|=E_ARDUINO_LOST;
		}
		return;
	}
	if(pos<=75){//den är på svart
		if(PIDoutput<-30)//om den styr åt vänster
		{
			memError+=1;
		}
	}else if(pos>=245){//den är på grönt
		if(PIDoutput>-10)//om den styr åt höger
		{
			memError-=5;
		}
	}else{
		memError=0;
	}
	int error = 160-pos;
	if(currentMillis-prevPIDMillis>1000)
	{
		errorPrior=error;
		memError=0;
		meanErrorPrior=(error+errorPrior)/2;
		integral=0;
	}
	//error+=memError; //kanske förstör för den deriverande delen
	prevPIDMillis=currentMillis;

	int meanError=(error+errorPrior)/2;

	integral=integral+error+memError;

	if(integral>maxPID)
	{
		integral=maxPID;
		if(currentMillis-prevMillis>500)
		{
			prevMillis=currentMillis;
			//BT_println("max");
		}
	}
	if(integral<-10000.0)
	{
		integral=-10000.0;
		if(currentMillis-prevMillis>500)
		{
			prevMillis=currentMillis;
			//BT_println("min");
		}
	}

	if(meanError-meanErrorPrior>NOISE_TOLERANCE||meanError-meanErrorPrior<-NOISE_TOLERANCE)
	{
		derivative = (double)(meanError - meanErrorPrior);
	}else{
		if(pos>180)
			derivative*=0.993;
		else if(pos<140)
			derivative*=0.998;
		else
			derivative*=0.9;
	}
	if(derivative>0)
	{
		PIDoutput = KP_Left*(error+memError-10) + KI_Left*integral + KD_Left*((double)(error - errorPrior))-20;// + KD*derivative*derivative; //+bias
		Doutput = KE_Left*derivative*derivative+10;
	}
	else
	{
		PIDoutput = KP_Left*(error+memError-10) + KI_Left*integral + KD_Left*((double)(error - errorPrior))-20;// - KD*derivative*derivative;
		Doutput = - KE_Left*derivative*derivative-5;
	}
	errorPrior=error;
	meanErrorPrior=meanError;
	//den deriverande ska öka och minska motor_L
	if(PIDoutput>30)
	{
		PIDoutput=30;
	}
	if(PIDoutput<-70)
	{
		PIDoutput=-70;
	}
	if(Doutput>50) PIDoutput=50;
	if(Doutput<-50)PIDoutput=-50;

	steer4(PIDoutput,50,-80,70,Doutput);
	/*if(currentMillis-prevMillis>500)
	{
		prevMillis=currentMillis;
		BT_printint(PIDoutput);
	}*/
}
void lineFollowRegleringRight()
{
	uint32_t currentMillis=millis();
	//uint32_t iterationTime = currentMillis-prevPIDMillis;

	int pos = RX2buffer[1];
	if(pos<=75){//den är på svart
		if(PIDoutput<-10)//om den styr åt vänster
			memError+=5;
	}else if(pos>=245){//den är på grönt
		if(PIDoutput>10)//om den styr åt höger
			memError-=5;
	}else{
		memError=0;
	}
	int error = 160-pos;
	if(currentMillis-prevPIDMillis>1000)
	{
		errorPrior=error;
		memError=0;
		meanErrorPrior=(error+errorPrior)/2;
		integral=0;
	}
	//error+=memError; //kanske förstör för den deriverande delen
	prevPIDMillis=currentMillis;

	int meanError=(error+errorPrior)/2;

	integral=integral+error+memError;

	if(integral>maxPID)
	{
		integral=maxPID;
		if(currentMillis-prevMillis>500)
		{
			prevMillis=currentMillis;
			//BT_println("max");
		}
	}
	if(integral<minPID)
	{
		integral=minPID;
		if(currentMillis-prevMillis>500)
		{
			prevMillis=currentMillis;
			//BT_println("min");
		}
	}
	if(meanError-meanErrorPrior>NOISE_TOLERANCE||meanError-meanErrorPrior<-NOISE_TOLERANCE)
	{
		derivative = (double)(meanError - meanErrorPrior);
	}else{
		if(pos>180)
			derivative*=0.993;
		else if(pos<140)
			derivative*=0.998;
		else
			derivative*=0.8;
	}
	if(derivative>0)
	{
		PIDoutput = KP_Right*(error+memError) + KI_Right*integral + KD_Right*((double)(error - errorPrior))+10;// + KD*derivative*derivative; //+bias
		Doutput = KE_Right*derivative*derivative;
	}
	else
	{
		PIDoutput = KP_Right*(error+memError) + KI_Right*integral + KD_Right*((double)(error - errorPrior))+10;// - KD*derivative*derivative;
		Doutput = - KE_Right*derivative*derivative;
	}
	errorPrior=error;
	meanErrorPrior=meanError;
	//den deriverande ska öka och minska motor_L
	if(PIDoutput>40) PIDoutput=40;
	if(PIDoutput<-20)PIDoutput=-20;
	if(Doutput>50) PIDoutput=50;
	if(Doutput<-50)PIDoutput=-50;

	steer4(PIDoutput,40,20,80,Doutput);
}
void lineFollowReglering()
{
	uint32_t currentMillis=millis();
	//uint32_t iterationTime = currentMillis-prevPIDMillis;

	int pos = RX2buffer[1];
	if(pos<=75){//den är på svart
		if(PIDoutput<-10)//om den styr åt vänster
			memError+=5;
	}else if(pos>=245){//den är på grönt
		if(PIDoutput>10)//om den styr åt höger
			memError-=5;
	}else{
		memError=0;
		integral=0;
	}
	int error = 160-pos;
	if(currentMillis-prevPIDMillis>1000)
	{
		errorPrior=error;
		memError=0;
		meanErrorPrior=(error+errorPrior)/2;
		integral=0;
	}
	//error+=memError; //kanske förstör för den deriverande delen
	prevPIDMillis=currentMillis;

	int meanError=(error+errorPrior)/2;

	integral=integral+error+memError;

	if(integral>maxPID)
	{
		integral=maxPID;
		if(currentMillis-prevMillis>500)
		{
			prevMillis=currentMillis;
			//BT_println("max");
		}
	}
	if(integral<minPID)
	{
		integral=minPID;
		if(currentMillis-prevMillis>500)
		{
			prevMillis=currentMillis;
			//BT_println("min");
		}
	}
	if(meanError-meanErrorPrior>NOISE_TOLERANCE||meanError-meanErrorPrior<-NOISE_TOLERANCE)
	{
		derivative = (double)(meanError - meanErrorPrior);
	}else{
		if(pos>180)
			derivative*=0.993;
		else if(pos<140)
			derivative*=0.998;
		else
			derivative*=0.8;
	}
	if(derivative>0)
	{
		PIDoutput = KP*(error+memError) + KI*integral + KD*((double)(error - errorPrior));// + KD*derivative*derivative; //+bias
		Doutput = KE*derivative*derivative;
	}
	else
	{
		PIDoutput = KP*(error+memError) + KI*integral + KD*((double)(error - errorPrior));// - KD*derivative*derivative;
		Doutput = - KE*derivative*derivative;
	}
	errorPrior=error;
	meanErrorPrior=meanError;
	//den deriverande ska öka och minska motor_L
	if(PIDoutput>80) PIDoutput=80;
	if(PIDoutput<-80)PIDoutput=-80;
	if(Doutput>50) PIDoutput=50;
	if(Doutput<-50)PIDoutput=-50;

	steer4(PIDoutput,40,0,50,Doutput);
	/*if(currentMillis-prevMillis>500)
	{
		prevMillis=currentMillis;
		BT_printint(PIDoutput);
	}*/
}
void lineFollowRegleringForward2()
{
	uint32_t currentMillis=millis();
	//uint32_t iterationTime = currentMillis-prevPIDMillis;


	int pos = RX2buffer[1];
	if(pos<=75){//den är på svart
		if(PIDoutput<-10)//om den styr åt vänster
			memError+=5;
	}else if(pos>=245){//den är på grönt
		if(PIDoutput>10)//om den styr åt höger
			memError-=pos-235;
	}else{
		memError=0;
	}
	int error = 160-pos;
	if(currentMillis-prevPIDMillis>1000)
	{
		errorPrior=error;
		memError=0;
		meanErrorPrior=(error+errorPrior)/2;
		integral=0;
	}
	//error+=memError; //kanske förstör för den deriverande delen
	prevPIDMillis=currentMillis;

	int meanError=(error+errorPrior)/2;

	integral=integral+error+memError;

	if(integral>maxPID)
	{
		integral=maxPID;
		if(currentMillis-prevMillis>500)
		{
			prevMillis=currentMillis;
			//BT_println("max");
		}
	}
	if(integral<minPID)
	{
		integral=minPID;
		if(currentMillis-prevMillis>500)
		{
			prevMillis=currentMillis;
			//BT_println("min");
		}
	}
	if(meanError-meanErrorPrior>NOISE_TOLERANCE||meanError-meanErrorPrior<-NOISE_TOLERANCE)
	{
		derivative = (double)(meanError - meanErrorPrior);
	}else{
		if(pos>180)
			derivative*=0.993;
		else if(pos<140)
			derivative*=0.998;
		else
			derivative*=0.8;
	}
	if(derivative>0)
	{
		PIDoutput = KP_forward*(error+memError) + KI_forward*integral + KD_forward*((double)(error - errorPrior));// + KD*derivative*derivative; //+bias
		Doutput = KE_forward*derivative*derivative;
	}
	else
	{
		PIDoutput = KP_forward*(error+memError) + KI_forward*integral + KD_forward*((double)(error - errorPrior));// - KD*derivative*derivative;
		Doutput = - KE_forward*derivative*derivative;
	}
	errorPrior=error;
	meanErrorPrior=meanError;
	//den deriverande ska öka och minska motor_L
	if(PIDoutput>40) PIDoutput=40;
	if(PIDoutput<-40)PIDoutput=-40;
	if(Doutput>50) PIDoutput=50;
	if(Doutput<-50)PIDoutput=-50;

	steer3(PIDoutput,50,30,Doutput);
}
void lineFollowRegleringForward()
{
	uint32_t currentMillis=millis();
	//uint32_t iterationTime = currentMillis-prevPIDMillis;

	prevPIDMillis=currentMillis;
	int pos = RX2buffer[1];
	if(pos<=70){
		if(PIDoutput<-10)//om den styr åt höger
			memError+=2;
	}else if(pos>=250){
		if(PIDoutput>10)//om den styr åt höger
			memError+=10;
	}else{
		memError=0;
	}
	int error = 160-pos+memError;


	integral=integral+(error);

	/*if(currentMillis-prevMillis>500)
	{
		prevMillis=currentMillis;
		BT_printint(pos);

	}*/
	if(integral>maxPID)
	{
		integral=maxPID;
		//if(currentMillis-prevMillis>500)
			//BT_println("max");
	}
	if(integral<minPID)
	{
		integral=minPID;
		//if(currentMillis-prevMillis>500)
			//BT_println("min");
	}
	derivative = (double)(error - errorPrior);

	PIDoutput = KP1*error + KI1*integral + KD1*derivative; //+bias
	errorPrior=error;

	if(PIDoutput>40) PIDoutput=40;
	if(PIDoutput<-40)PIDoutput=-40;
	steer(PIDoutput,50,30);
}

void regulateStanding() //Test för att regulera rotationen, används ej i slutgiltigt program
{
	//double KP = 0.4128;
	//double KI = 2.0592e-11;
	//double KD = 9.8928e+10
	//int m = 85.0;

	uint32_t currentMillis=millis();
	//uint32_t iterationTime = currentMillis-prevPIDMillis;
	prevPIDMillis=currentMillis;
	int pos = RX2buffer[1];
	if(pos<=70){
		if(PIDoutput<-10)//om den styr åt höger
			memError+=2;
	}else if(pos>=250){
		if(PIDoutput>10)//om den styr åt höger
			memError+=10;
	}else{
		memError=0;
	}
	int error = 160-pos+memError;


	integral=integral+(error);

	/*if(currentMillis-prevMillis>500)
	{
		prevMillis=currentMillis;
		BT_printint(pos);

	}*/
	if(integral>maxPID)
	{
		integral=maxPID;
		//if(currentMillis-prevMillis>500)
			//BT_println("max");
	}
	if(integral<minPID)
	{
		integral=minPID;
		//if(currentMillis-prevMillis>500)
			//BT_println("min");
	}
	derivative = (double)(error - errorPrior);

	PIDoutput = 0.5*(KP*error + KI*integral + KD*derivative); //+bias
	errorPrior=error;


	if(PIDoutput>40) PIDoutput=40;
	if(PIDoutput<-40)PIDoutput=-40;
	steer(PIDoutput,0,-100);
}
void steer2(int amount)// amount mellan-50 och 50 R+L=100 alltid
{

	if(amount>=0)
	{
		motor_L(amount);
		motor_R(0);
	}else
	{
		motor_L(0);
		motor_R(-amount);
	}
}
void steer(int amount, int midlevel, int minimumlevel)// amount mellan-50 och 50 R+L=100 alltid
{ //positiv amount styr åt höger
	if(amount>100-midlevel)
	{
		motor_L(100);
	}else
	{
		if(midlevel+amount<minimumlevel)
			motor_L(minimumlevel);
		else
			motor_L(midlevel+amount);
	}
	if(amount<-100+midlevel)
	{
		motor_R(100);
	}else{
		if(midlevel-amount<minimumlevel)
			motor_R(minimumlevel);
		else
			motor_R(midlevel-amount);
	}

}
void steer3(int amount, int midlevel, int minimumlevel, int ajustL)// amount mellan-50 och 50 R+L=100 alltid
{ //positiv amount styr åt höger
	int R_val;
	int L_val;
	if(amount>100-midlevel)
	{
		L_val=100;
	}else
	{
		if(midlevel+amount<minimumlevel)
			L_val=minimumlevel;
		else
			L_val=midlevel+amount;
	}
	if(amount<-100+midlevel)
	{
		R_val=100;
	}else{
		if(midlevel-amount<minimumlevel)
			R_val=minimumlevel;
		else
			R_val=midlevel-amount;
	}
	if(L_val>minimumlevel-10&&L_val<100)
	{
		if(ajustL>0)
			L_val+=3.0*ajustL;
		else
			L_val+=ajustL;
	}
	if(L_val>100)L_val=100;
	else if(L_val<-100)L_val=-100;
	if(R_val>100)R_val=100;
	else if(R_val<-100)R_val=-100;
	motor_R(R_val);
	motor_L(L_val);

}
void steer4(int amount, int midlevel, int minimumlevel, int maxlevel, int ajustL)// amount mellan-50 och 50 R+L=100 alltid
{ //positiv amount styr åt höger
	int R_val;
	int L_val;
	if(amount>100-midlevel)
	{
		L_val=100;
	}else
	{
		if(midlevel+amount<minimumlevel)
			L_val=minimumlevel;
		else
			L_val=midlevel+amount;
	}
	if(amount<-100+midlevel)
	{
		R_val=100;
	}else{
		if(midlevel-amount<minimumlevel)
			R_val=minimumlevel;
		else
			R_val=midlevel-amount;
	}
	if(L_val>minimumlevel-10&&L_val<100)
	{
		if(ajustL>0)
			L_val+=3.0*ajustL;
		else
			L_val+=ajustL;
	}
	if(L_val>maxlevel)L_val=maxlevel;
	else if(L_val<-maxlevel)L_val=-maxlevel;
	if(R_val>maxlevel)R_val=maxlevel;
	else if(R_val<-maxlevel)R_val=-maxlevel;
	motor_R(R_val);
	motor_L(L_val);

}

void lineFollowForward(){
	if(binaryColor(RX2buffer[1])=='b')
	{
		if(lastColor!='b')
		{
			timeFlag2=millis();
			if(printDebugInfo) BT_println("b");
		}
			if(millis()-timeFlag1>2500)
			{  //minska ner ytterligare om det tar lång tid
				motor_L(40);
				motor_R(10);
			}
			else if(millis()-timeFlag1>3000)
			{  //minska ner ytterligare om det tar lång tid
				motor_L(40);
				motor_R(-10);
			}else if(timeFlag2-timeFlag1<400)
			{
				/*if(130 < RX2buffer[1] && RX2buffer[1] < 150){
					motor_L(70);
					motor_R(70);
				}else{
					motor_L(45);
					motor_R(38);
				}*/
				lineFollowRegleringForward();
			}else{
				motor_L(45);
				motor_R(38);
			}
		lastColor='b';
	}else if(binaryColor(RX2buffer[1])=='w')
	{
		if(lastColor!='w')
		{
			timeFlag1=millis();
			if(printDebugInfo) BT_println("w");
		}

			if(millis()-timeFlag2>3000)
			{  //minska ner ytterligare om det tar lång tid
				motor_L(-10);
				motor_R(40);
			}
			else if(timeFlag1-timeFlag2<400)
			{
				/*
				if(125 < RX2buffer[1] && RX2buffer[1] < 155)
				{
					motor_L(70);
					motor_R(70);
				}else{
					motor_L(25);
					motor_R(45);
				}*/
				lineFollowRegleringForward();
			}else if(millis()-timeFlag1>2000)
			{
				motor_L(-40);
				motor_R(40);
			}else{
				motor_L(25);
				motor_R(45);
			}

		lastColor='w';
	}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */ //Klock initiering genererat av MXCube
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */ //I2C initiering genererat av MXCube
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */ //I2C initiering genererat av MXCube
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */ //Timer initiering genererat av MXCube. Används för PWM i motorer
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */ //Timer initiering genererat av MXCube
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}
/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */ //Timer initiering genererat av MXCube
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 32000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 800;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}
/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */ //UART init generarat av MXCube
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}
/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */ //UART init genererat av MXCube
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
//GPIO init genererat av MXCube
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_B_Pin|DIR_A_Pin|shutdown3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, test_led_Pin|shutown2_Pin|shutdown1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USER_BUTTON_Pin SPEED1_Pin SPEED2_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin|SPEED1_Pin|SPEED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_B_Pin DIR_A_Pin shutdown3_Pin */
  GPIO_InitStruct.Pin = DIR_B_Pin|DIR_A_Pin|shutdown3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : test_led_Pin shutown2_Pin shutdown1_Pin */
  GPIO_InitStruct.Pin = test_led_Pin|shutown2_Pin|shutdown1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
// GPIO external interrupt, (Debug knapp och Varvtalssensorer)
void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin)
{
	if(GPIO_Pin == USER_BUTTON_Pin&&(millis()-prevMillis>debounceInterval))
	//if(1)
	{
		prevMillis=millis();
		switch(buttonCounter)
		{
		case 0:
			BT_println("rst cntR");
			reset_counter_R();
			break;
		case 1:
			//BT_println("Button pressed");
			//BT_printdouble(KP);

			//Atmel_printchar("<");
			break;
		case 2:
			BT_printint(read_dist_R());
			break;
		case 3:
			break;
		case 4:
			buttonCounter=0-1;//obs ++ senare
			break;
		}
		buttonCounter++;
	}
	else if(GPIO_Pin == SPEED1_Pin) //Vänster varvtal har triggats
	{
		//HAL_GPIO_TogglePin(test_led_GPIO_Port,test_led_Pin); används för felsökning
		speed_L_ISR();
	}
	else if(GPIO_Pin == SPEED2_Pin) //Höger varvtal har triggats
	{
		speed_R_ISR();
	}
}
//Kopierar en array in i TXbuffer
void copyToTXbuffer(char* a)
{
	int i;
	for(i=0; a[i]!='\0'&&a[i]!='\n'; i++)
	{
		TXbuffer[i]=a[i];
	}
	for(; i<TX_BUF_SIZE-1; i++)
	{
		TXbuffer[i]=' ';
	}
	TXbuffer[TX_BUF_SIZE-1]='\n';

}
//Skicka en rad till Arduinon
void Atmel_println(char* s)
{
	int c = 0;
	while (s[c] != '\0'&&c<TX2_BUF_SIZE-1) {
		TX2buffer[c] = s[c];
		c++;
	}
	while(c<TX2_BUF_SIZE-1){
		TX2buffer[c]=' ';
		c++;
	}
	TX2buffer[TX2_BUF_SIZE-1]='\n';

	HAL_UART_Transmit_IT(&huart1, (uint8_t *)TX2buffer,TX2_BUF_SIZE);
}
//skicka en char till Arduinon
void Atmel_printchar(char* s)
{
	TX2buffer[0]=s[0];
	HAL_UART_Transmit_IT(&huart1, s,1);
}

//Skickar värdet av en double som en String via BT
void BT_printdouble(double d)
{
	char out[15];
	//float_to_string(d,out);
	double_to_char(d,out);
	BT_println(out);
}

//konverterar doule till en String
void double_to_char(double f,char * buffer){
    gcvt(f,10,buffer);
}

//Skickar värdet av en int som en String via BT
void BT_printint(int a)
{
	const int maxN = 999999;
	int c = 0;
	if(a<0)
	{
		a=-a;
		TXbuffer[0]='-';
	}else{
		TXbuffer[0]='+';
	}c++;
	if(a>maxN) a=maxN;
	for(int i = maxN+1; i>0; i/=10){
		int currentDigit=a/i;
		if(currentDigit>0)
		{//printa tusental
			TXbuffer[c] = currentDigit+'0';
			a=a-currentDigit*i;
		}else{
			TXbuffer[c] = '0';
		}
		c++;
	}
	while(c<TX_BUF_SIZE-1){
		TXbuffer[c]=' ';
		c++;
	}
	TXbuffer[TX_BUF_SIZE-1]='\n';
	HAL_UART_Transmit_IT(&huart6, (uint8_t *)TXbuffer,TX_BUF_SIZE);
}
#define MAX_WAIT_TIME 100

//Be om nytt meddelande från överordnad dator
void BT_wtf()
{
	HAL_UART_AbortReceive_IT(&huart6);

	errorInt&=~((uint8_t)E_BT_NOT_AVAILABLE);

	int c = 0;
	tempTXbuffer[c++]='+';
	while(c<TX_BUF_SIZE-1){
		tempTXbuffer[c]='_';
		c++;
	}
	tempTXbuffer[TX_BUF_SIZE-1]='\n';
	if(HAL_UART_Transmit_IT(&huart6, (uint8_t *)tempTXbuffer,TX_BUF_SIZE)==HAL_BUSY){
		errorInt|=E_BT_NOT_AVAILABLE;
		HAL_UART_AbortReceive_IT(&huart6);
		HAL_UART_AbortTransmit_IT(&huart6);
	}else{
		errorInt&=~E_BT_NOT_AVAILABLE;
		//ready=1;
	}
}
//skickar en angiven string via Bluetooth UART i korrekt format
void BT_println(char* s)
{
	if(s[0]=='+')
	{
		BT_wtf(); // "+" ska ej sparas i TXbuffer för att förhindra låsning
	}else{
	//uint32_t start=millis(); debug
	//uint8_t ready = 0; debug

	errorInt&=~((uint8_t)E_BT_NOT_AVAILABLE);

	int c = 0;
	while (s[c] != '\0'&&c<TX_BUF_SIZE-1) { //fram tills slutet av string s
		TXbuffer[c] = s[c]; //lägg in meddelandet i TXbuffer
		c++;
	}
	while(c<TX_BUF_SIZE-1){//hela buffern ska vara fylld
		TXbuffer[c]=' ';
		c++;
	}
	TXbuffer[TX_BUF_SIZE-1]='\n'; //sista tecknet är end of line
	//int MAX_K = 10;
	//int k=0;
	//while(k<MAX_K){//millis()-start<MAX_WAIT_TIME&&(!ready) debug
		if(HAL_UART_Transmit_IT(&huart6, (uint8_t *)TXbuffer,TX_BUF_SIZE)==HAL_BUSY){
			errorInt|=E_BT_NOT_AVAILABLE;
			//HAL_UART_AbortReceive_IT(&huart6);
			HAL_UART_AbortTransmit_IT(&huart6);
		}else{
			errorInt&=~E_BT_NOT_AVAILABLE;
			//ready=1; debug
		}
		//k++;}
	}
}
//skickar senaste meddelandet igen
void BT_repeat()
{
	HAL_UART_Transmit_IT(&huart6, (uint8_t *)TXbuffer,TX_BUF_SIZE);
}
//Interrupt som triggas när ett meddelande mottagits via Bluetooth
void KTS_message_recieved(UART_HandleTypeDef *huart)
{
	if(connected){ //utför bara om uppkopplingen är godkänd
	newMessage=1; //meddelandet i RXbuffer är nu nytt
	for(int i=0; i<RX_BUF_SIZE-1;i++)//kolla så det inte är något fel
	{
		if(RXbuffer[i]=='\n'||RXbuffer[i]=='\0')
		{
			RX_error|=1;//meddelandet är för kort eller förvrängt
			//clearRXbuffer();
			BT_println("+");
			return;
		}
	}
	if(RXbuffer[RX_BUF_SIZE-1]!='\n')//meddelandet är för långt
	{
		RX_error|=1;
		//clearRXbuffer();
		BT_println("+");
		return;
	}
	uint8_t roger = 1;//sätts till 0 om meddelandet inte kan tolkas
	if(RXbuffer[0]=='+'){//datorn ber om repetition av föregående meddelande
			BT_repeat();
			roger = 0;
	}else if(RXbuffer[0]=='?'){//Fråga om uppkoppling, svara med "!"
		BT_println("!");
		roger = 0;
	}else if(RXbuffer[0]=='L'){//tänd debug LED
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
	}else if(RXbuffer[0]=='T'){//välj vilken väg fordonet ska ta
		//detta hanteras utanför denna interrupt med newOrder
		if(RXbuffer[1]=='<'){//vänster
			newOrder = '<';
		}else if(RXbuffer[1]=='>'){//höger
			newOrder = '>';
		}else if(RXbuffer[1]=='^'){//fram
			newOrder = '^';
		}else if(RXbuffer[1]=='u'){//u-sväng
			newOrder = 'u';
		}
		if(clearOrder)
		{
			newOrder=0;
			clearOrder=0;
		}
	}else if(RXbuffer[0]=='R'){ //Remote controll
		driveState=8;
		stateChanged=1;

	}else if(RXbuffer[0]=='S'){ //stanna bilen
		//driveforward(0);
		driveState=0;
		stateChanged=1;
	}else if(RXbuffer[0]=='j'){ //justera regleringsvariabel
								//används för att tweaka PID
		roger=0;//skicka inte "ok" här
		if(RXbuffer[1]=='P'){
			adjustThis=&KP;
		}else if(RXbuffer[1]=='I'){
			adjustThis=&KI;
		}else if(RXbuffer[1]=='D'){
			adjustThis=&KD;
		}else if(RXbuffer[1]=='E'){
			adjustThis=&KE;
		}
		if(RXbuffer[2]=='*') //öka den angivna parametern ett steg
		{
			//*adjustThis=(*adjustThis)*((double)(RXbuffer[3]-'0'));
			*adjustThis=(*adjustThis)*(1.1);
		}else if(RXbuffer[2]=='/')//minska den angivna parametern ett steg
		{
			//*adjustThis=(*adjustThis)/((double)(RXbuffer[3]-'0'));
			*adjustThis=(*adjustThis)/(1.1);
		}
		BOOL5_TRUE;
		timeFlag3=millis();

	}else if(RXbuffer[0]=='\n'||RXbuffer[0]=='\r'){//ej tillåtet på position 0
		//buffer is offset
		RX_error|=1;
		BT_println("+");
		roger = 0;
	}else{ //meddelanden som ej finns i protokoll
		RX_error|=1;
		BT_println("+");
		roger = 0;
	}
	if(roger){//meddelandet har tolkats, skicka ok
		BT_println("ok");
	}
}}
// Skicka RFID taggens nummer som en string, Ex: "A53"
void send_RFID_info(uint8_t RFIDindex)
{
	if(RFIDindex!=lastRFIDindex){
		char RFIDmessage[TX_BUF_SIZE];
		int i = 0;
		uint8_t roadType = RFIDindex/10;
		uint8_t roadIndex = RFIDindex-roadType*10;
		RFIDmessage[i++]='A';
		RFIDmessage[i++]='0'+roadType;
		RFIDmessage[i++]='0'+roadIndex;
		while(i<TX_BUF_SIZE-1)
		{
			RFIDmessage[i++]=' ';//fyll resten av bufferten med mellanrum
		}
		RFIDmessage[TX_BUF_SIZE-1]='\n'; //slutet på meddelandet

		copyToTXbuffer(RFIDmessage); //spara meddelandet i TXbuffer
		HAL_UART_Transmit_IT(&huart6, (uint8_t *)TXbuffer,TX_BUF_SIZE);//skicka
		lastRFIDindex=RFIDindex;
	}
}
//Utför beslut efter RFID nr
void RFID_decision(uint8_t RFIDindex)
{
	if(RFIDindex!=0){
		uint8_t roadType = RFIDindex/10;
		uint8_t roadIndex = RFIDindex-roadType*10;
		if(roadType!=lastRoadType||roadIndex!=lastRoadIndex||continueRutt){
			switch(roadType)
			{
			case 1://rak väg
				//clearOrder=1;
				driveState = LINE_FOLLOW;
				stateChanged = 1;
				break;
			case 2://vänstersväng
				//clearOrder=1;
				driveState = 11;//left turn
				stateChanged = 1;
				break;
			case 3://högersväng
				//clearOrder=1;
				driveState = LINE_FOLLOW;
				stateChanged = 1;
				break;
			case 4://T-korsning höger eller fram
				driveState = WAIT_FOR_ORDER;
				stateChanged = 1;
				break;
			case 5://T-korsning höger eller vänster
				driveState = WAIT_FOR_ORDER;
				stateChanged = 1;
				break;
			case 6://T-korsning vänster eller fram
				driveState = WAIT_FOR_ORDER;
				stateChanged = 1;
				break;
			case 7://4v-korsning
				driveState = WAIT_FOR_ORDER;
				stateChanged = 1;
				break;
			}
			globalRoadType=roadType;//spara roadType i en global variabel för linjeföljningen
			continueRutt=0;
		}
		lastRoadType = roadType;
		lastRoadIndex = roadIndex;
	}
}
//Interrupt då Atmegan har skickat meddelande
void Atmel_message_recieved(UART_HandleTypeDef *huart)
{
	// RX2buffer: [RFID nr, färgsensor1, färgsensor2, endline(\n)]
	for(int i=0; i<RX2_BUF_SIZE-1;i++)
	{
		if(RX2buffer[i]=='\n')
		{//om \n finns på fel plats, rensa buffern och be om nytt meddelande
			RX_error|=2;
			Atmel_printchar("+");
			return;
		}
	}
	if(RX2buffer[0]>0){ //RFID närvarande
		send_RFID_info(RX2buffer[0]); //skicka taggens nr till datorn
		if(obeyRFID)
			RFID_decision(RX2buffer[0]); //utför beslut utefter tagg
	}
}
// UART message received, interrupt som triggas vid mottaget meddelande
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart6)
	{
		KTS_message_recieved(&huart6);

	}else if(huart==&huart1)
	{
		Atmel_message_recieved(huart);
	}else
	{
		RX_error=3;
	}
}
// interrupt som triggar när transmit via UART är klar
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if((huart==&huart6)&&((RX_error)&((uint8_t)1))){ //om det finns ett error i UART6 (Bluetooth)
		clearRXbuffer(&huart6); //rensa RXbuffern
		RX_error&=~((uint8_t)1);
	}
	if(RX_error&2){	//om det finns ett error i UART1 (Arduino)
		clearRXbuffer(&huart1);
		RX_error&=~((uint8_t)2);
	}
	if(TXbuffer[0]=='i') connected=1; //uppkoppling har precis gjorts och init meddelande är skickat
}
//Interrupt vid error i UART.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	//vid error i kommunikationen, dela info med datorn
	if(huart==&huart1)
	{
		if(huart1.ErrorCode==HAL_UART_ERROR_PE) 		BT_println("E21P");	/*!< UART1 Parity error        */
		else if(huart1.ErrorCode==HAL_UART_ERROR_NE)	BT_println("E21N");	/*!< UART1 Noise error         */
		else if(huart1.ErrorCode==HAL_UART_ERROR_FE)	BT_println("E21F");	/*!< UART1 Frame error         */
		else if(huart1.ErrorCode==HAL_UART_ERROR_ORE)	BT_println("E21O");	/*!< UART1 Overrun error       */
		else if(huart1.ErrorCode==HAL_UART_ERROR_DMA)	BT_println("E21D");	/*!< UART1 DMA transfer error  */
	}
	else if(huart==&huart6)
	{
		if(huart1.ErrorCode==HAL_UART_ERROR_PE) 		BT_println("E26P");	/*!< UART6 Parity error        */
		else if(huart1.ErrorCode==HAL_UART_ERROR_NE)	BT_println("E26N");	/*!< UART6 Noise error         */
		else if(huart1.ErrorCode==HAL_UART_ERROR_FE)	BT_println("E26F");	/*!< UART6 Frame error         */
		else if(huart1.ErrorCode==HAL_UART_ERROR_ORE)	BT_println("E26O");	/*!< UART6 Overrun error       */
		else if(huart1.ErrorCode==HAL_UART_ERROR_DMA)	BT_println("E26D");	/*!< UART6 DMA transfer error  */
	}
}
//säger om den är innanför eller utanför linjen:
char binaryColor(uint8_t val){

	if(val<colorLimit)//140 på c gjordes i de första testerna
	{
		if(lastReadColor=='w')//går från vit till svart
		{
			colorLimit=160; // Schmitt-trigger för att reducera brus
			lastReadColor='b';
		}
		return 'b';
	}else{
		if(lastReadColor=='b')//går från vit till svart
		{
			colorLimit=135; // Schmitt-trigger för att reducera brus
			lastReadColor='w';
		}
		return 'w';
	}
}
//återställer mottagningsbufferten vid fel så ett nytt meddelande kan läsas in
void clearRXbuffer(UART_HandleTypeDef *huart)
{
	int RXbufferSize;
	int TXbufferSize;
	char *tempRXbufferPointer;
	char *tempTXbufferPointer;
	//Avbryt eventuellt pågående meddelande
	HAL_UART_AbortReceive_IT(huart);
	__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);

	char temp[TX_BUF_SIZE];
	if(huart==&huart1)
	{
		//TXbufferSize = TX2_BUF_SIZE;
		RXbufferSize = RX2_BUF_SIZE;
		tempRXbufferPointer=RX2buffer;
	}else if(huart==&huart6)
	{
		RXbufferSize = RX_BUF_SIZE;
		TXbufferSize = TX_BUF_SIZE;
		tempRXbufferPointer=RXbuffer;
		//store TXbuffer in temp array
		for(int i=0; i<TXbufferSize;i++)
			temp[i]=TXbuffer[i];
	}
	//töm UART register och resetta
	__HAL_UART_FLUSH_DRREGISTER(huart);

	//fill with clear chars
	for(int i=0; i<RXbufferSize-1;i++)
		tempRXbufferPointer[i]=' ';
	tempRXbufferPointer[RXbufferSize-1]='\n';
	if(huart==&huart6)
	{
		//restore TXbuffer
		for(int i=0; i<TXbufferSize;i++)
			TXbuffer[i]=temp[i];
	}
	//återaktivera Interrupt för den angivna uart
	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
	//HAL_UART_Receive_IT(huart, (uint8_t *)tempRXbufferPointer, RXbufferSize);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{
		HAL_GPIO_TogglePin(test_led_GPIO_Port, test_led_Pin);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
