/*
 * utils.h
 *
 *  Created on: Dec 27, 2023
 *      Author: kara
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

//All Libraries used in VCU Project
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "stdio.h"
#include "stdlib.h"
#include "usart.h"
#include "gpio.h"

#define CALIBRATE 0

#define MAX_PACK_VOLTAGE 600.0

//ADC MACROS

#define MOVING_AVERAGE 20

#define ADC_MAX_VALUE 4095

//#define SOFTWARE_BSPD 1

//APPS MACROS

#define APPS1_MIN_DEFAULT 100
#define APPS1_MAX_DEFAULT 4000
#define APPS2_MIN_DEFAULT 110
#define APPS2_MAX_DEFAULT 4110
#define BRAKESENSOR_MIN_DEFAULT 100
#define BRAKESENSOR_MAX_DEFAULT 4000

//EEPROM MACROS

#define EEPROM_STARTING_ID (uint32_t)0x10000

//DISPLAY MACROS

#define DISPLAY_DELAY 3

//INVERTER MACROS

#define MAX_RPM 6000

#define PI 3.141592

#define WHEEL_RADIUS 12

#define GEAR_RATIO 3.98

#define BUZZER_DELAY 2000

#define INVERTER_DELAY 20
typedef enum{
	INVERTER_ACTUAL_CURRENT_COMMAND = 0x20,
	INVERTER_ACTUAL_Q_CURRENT_COMMAND = 0x27,
	INVERTER_CURRENT_200_PC_COMMAND = 0xd9,
	INVERTER_SEND_TORQUE_COMMAND = 0x90,
	INVERTER_ASK_VALUE_COMMAND = 0x3d,
	INVERTER_CONTROL_MODE_COMMAND = 0x51,
	INVERTER_SPEED_COMMAND = 0xa8,
	INVERTER_MOTOR_TEMP_COMMAND = 0x49,
	INVERTER_IGBT_TEMP_COMMAND = 0x4a,
	INVERTER_CURRENT_1_AC_COMMAND = 0x54,
	INVERTER_CURRENT_2_AC_COMMAND = 0x55,
	INVERTER_CURRENT_3_AC_COMMAND = 0x56,
	INVERTER_DC_LINK_COMMAND = 0xeb
}inverterCANCommands;

typedef enum{
	INVERTER_ENABLE = 0x0,
	INVERTER_DISABLE = 0x4
}inverterModeCommands;

typedef enum{OFF_MODE = 0, RUN_MODE = 1}inverterState;

typedef enum { MAIN_PAGE = 0X0, ELECTRICAL_PAGE = 0X1, MECHANICAL_PAGE = 0x2, TEMPERATURES_PAGE = 0x3, DEBUG_PAGE = 0x4, DEVICES_PAGE = 0x5, MENU_PAGE = 0x6}pages;

typedef enum { OFF = 0X0, PRECHARGED = 0X1, R2D = 0x02}VCUstate;

typedef enum { NOT_PRESSED = 0X0, PRESSED = 0X1}button;

typedef enum { ENTERED = 0X0, TO_ENTER = 0X1}FirstLoopEnter;

typedef enum { DEVICE_DISCONNECTED = 0X0, DEVICE_IS_CONNECTED = 0X1}deviceState;


typedef enum {TIMER_RESET = 0x0, TIMER_TRIGGERED} timerState;

typedef enum {NONE_ERROR = 0x0, APPS_DIFF_ERROR, APPS_OUT_OF_RANGE_ERROR, BRAKESENSOR_OUT_OF_RANGE, INVERTER_DISCONNECTED} VCUlastError;

typedef struct CANData{
	CAN_TxHeaderTypeDef   TxHeader;
	uint8_t               TxData[8];
	uint32_t              TxMailbox;
	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t               RxData[8];
	uint8_t 			  checkSum;
}CANData;


typedef struct SensorData{
	int16_t mappedMin;
	int16_t mappedMax;
	int32_t sum;
	int16_t filtered;
	int16_t sampleArray[MOVING_AVERAGE];
}SensorData;

typedef struct Button{
	uint16_t pin;
	button state;
}Button;

typedef struct Pin{
	uint16_t pin;
	GPIO_TypeDef* group;
}Pin;

typedef struct Timer{
	TIM_HandleTypeDef *timer;
	uint8_t triggered;

}Timer;

typedef struct CellData{
	float val;
	uint8_t id;
//	CellID id; // 0 -12
//	Segment segment;

}CellData;


typedef struct VCUData{


	VCUstate state;

	VCUlastError lastError;

	pages menuPage;

	uint16_t maxTorque;

	uint16_t rawSensorData[3];
	uint16_t rawSensorDataADC1;
	uint16_t brakeR2DThreshold;
	uint16_t brakeLightThreshold;
	int16_t brakeSensorFinalValue;

	Pin brakeLight;
	Pin buzzer;

	Button utils1Button;
	Button utils2Button;
	Button tsActButton;
	Button R2DButton;

	uint16_t upperDeadZoneThreshold;
	uint16_t lowerDeadZoneThreshold;

	FirstLoopEnter loadNewScreen;
	FirstLoopEnter offStateFirstLoopEnter;
	FirstLoopEnter prechargedStateFirstLoopEnter;
	FirstLoopEnter R2DStateFirstLoopEnter;

	Timer ADCSampleRateTimer;
	Timer startStateTimer;

	int16_t pedalSensorFinalValue;
	SensorData brakeSensor;
	SensorData APPS1;
	SensorData APPS2;
	float brakePressure;
	uint16_t brakeSensorRange;


	uint16_t telemetryTimeoutFlag;
	uint8_t telemetryIsConnected;

}VCUData;

typedef struct ACUData{
	uint8_t isConnected;

	uint8_t userIsConnected;
	uint8_t userTimeOutFlag;

	uint8_t PREIsArmed;
	uint8_t	AIRMinusIsArmed;
	uint8_t	AIRPlusIsArmed;
	uint8_t SCSLEDsAreConnected;
	uint8_t AIRsAreStuck;
	uint8_t TSOver60Volt;

	uint8_t vicorTemperature;
	float humidity;
	uint8_t PCBTemperature;

	uint16_t SCSLEDsCheckSumCurrentErrors;

	uint8_t SCSLEDsCheckSumAcceptedErrors;
	uint8_t SCSLEDsTemporaryCheckSum;

	uint8_t highestCellTemp;
	uint16_t IMDResistance;

	uint8_t IMDHasError;
	uint8_t AMSHasError;
	uint8_t AMSHasLatchedError;

	Pin IMDLed;
	Pin AMSLed;

	// Variables for storing the max and min cell voltage of the pack.
	CellData minVoltageCell;
	CellData maxVoltageCell;

	// Variable for storing the max temperature among the cells.
	CellData maxCellTemp;

	Timer SCSLEDsTimeOutTimer;

	uint8_t soc;
}ACUData;

typedef struct InverterData{
	uint16_t timeoutFlag;
	uint8_t isConnected;
	uint8_t igbtTemp;
	uint8_t motorTemp;
	int16_t DCBus;
	int16_t RPMSpeed;
	int16_t current1AC;
	int16_t current2AC;
	int16_t current3AC;
	float actualCurrent;
	float actualCurrentQ;
	int16_t currentDevice;
	int16_t current200pc;
	Pin run;
	inverterState state;
	Timer timeOutTimer;
}InverterData;

typedef struct DataLoggerData{
	uint8_t arePoweredOn;

	uint8_t mainIsConnected;
	uint16_t mainTimeOutFlag;

	uint8_t controlBoxIsConnected;
	uint16_t controlBoxTimeOutFlag;

	uint8_t aeroIsConnected;
	uint16_t aeroTimeOutFlag;

	float vehicleSpeed;
	uint8_t coolingThermistors[6];
	float maxAcc;
	uint8_t SteeringAngle;
	uint16_t wheelRPM;
	uint16_t motorRPM;
	uint8_t wantedDutyCycle;

	uint8_t aeroDataLoggerMode;
	int16_t frontLeftWheelRPM;
	int16_t frontRightWheelRPM;
	int16_t rearLeftWheelRPM;
	int16_t rearRightWheelRPM;

	uint8_t minutes;
	uint8_t hours;

}DataLoggerData;


typedef struct IsabellenData{
	uint16_t timeoutFlag;
	uint8_t isConnected;
	float current;
	int16_t currentCounter;

	float voltage1;
	float voltage2;
	float voltage3;

	float wattage;
	int16_t wattageCounter;

}IsabellenData;

//Extern Peripherals
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

//Extern Structures
extern ACUData acu;
extern InverterData inverter;
extern IsabellenData ivt;
extern DataLoggerData dataLogger;
extern VCUData vcu;

//Functions
HAL_StatusTypeDef retrieveVCUDataFromEepromOnStartUp(I2C_HandleTypeDef *_eepromI2C, VCUData *_vcu);
HAL_StatusTypeDef VCUInit(VCUData *_vcu, TIM_HandleTypeDef *_startStateTimer,TIM_HandleTypeDef *_ADCSampleRateTimer,uint16_t _brakeLightThreshold, uint16_t _brakeR2DThreshold);
HAL_StatusTypeDef ACUInit(ACUData *_acu, TIM_HandleTypeDef *_SCStimeOutTimer, uint8_t _SCSLEDsCheckSumAcceptedErrors);
HAL_StatusTypeDef DataloggerInit(DataLoggerData *_datalogger);
HAL_StatusTypeDef InverterInit(InverterData *_inverter, CAN_HandleTypeDef *hcan, TIM_HandleTypeDef *_timeOutTimer);
HAL_StatusTypeDef IVTInit(IsabellenData *_ivt);
void devicesTimeOutTimers(VCUData *_vcu,ACUData *_acu, DataLoggerData *_datalogger, IsabellenData *_ivt);
int16_t getIntFromVoltage(float voltage);
float getVoltage(int data);
void writeDataEeprom(I2C_HandleTypeDef *_eepromI2C, uint32_t startingID, uint8_t *data, uint8_t length);
void readDataEeprom(I2C_HandleTypeDef *_eepromI2C, uint32_t startingID, uint8_t *data, uint8_t length);
void bar2Screen(UART_HandleTypeDef *_huart, char *ID, float val);
void error2Screen(UART_HandleTypeDef *_huart,char error[]);
void str2Screen(UART_HandleTypeDef *_huart, char *ID, char *string);
void int2Screen(UART_HandleTypeDef *_huart, char *ID, float val);
void changeTextColor2Screen(UART_HandleTypeDef *_huart,char obj[], char txt[]);
void changePage2Screen(UART_HandleTypeDef *_huart, char page[]);
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
void brakeSensor(VCUData *_vcu);
void pedalSensor(VCUData *_vcu);
void setTorque(CAN_HandleTypeDef *_hcan, int16_t trq);
void askInverterValue(CAN_HandleTypeDef *_hcan,uint8_t regid,uint8_t dt);
void activateInverter( CAN_HandleTypeDef *_hcan,InverterData *_inverter);
void deactivateInverter(CAN_HandleTypeDef *_hcan, InverterData *_inverter);
uint32_t EEPROM_MultiByte_Write(I2C_HandleTypeDef *hi2c, uint32_t mem_addr, uint8_t *pData, uint16_t size);
uint32_t EEPROM_MultiByte_Selective_Read(I2C_HandleTypeDef *hi2c, uint32_t mem_addr, uint8_t *pData, uint16_t size);
float getVoltage(int data);
#endif /* INC_UTILS_H_ */
