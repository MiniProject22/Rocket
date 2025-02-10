/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Calibrate 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define BUFFER_SIZE 90
char rx_buffer[BUFFER_SIZE];    // Buffer de réception
volatile uint8_t rx_index = 0;  // Index pour écrire dans le buffer
volatile bool message_ready = false; // Indique si un message complet est reçu
char received_char = 'Z';
#define MPU6050_ADDR 0x68 << 1  // Adresse I2C du MPU6050 (décalée pour HAL)
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	// Entrance Angle
	float angleO = 0; // En radiant!!! Le petit
	float angleP = 0; // En radiant!!! Le tour

	// Step Freq Const
	float Current_Max_Freq = 0; // Hz
	float Max_Start_Freq = 1000; // Hz
	float Max_End_Freq = 1800; // Hz
	float Max_Accel = 50; // Hz/s
	float Nb_Acc_p_action_accél = 1; // Hz
	int Presc_a, Presc_b;

	// STM32 data Const
	int PWM_Freq = 80000000;
	int PWM_Counter_Period = 401; // +1
	double Precision_taille_verin = 0.001f; // m

	// Coordonate
	double CoordA[3] = {}, CoordB[3] = {};
	double CoordAP[3] = {}, CoordBP[3] = {};

	// Angular velocity
	double Rota_SpeedA = 0, Rota_SpeedB = 0;

	// Lenght actuator
	double LenghtA, LenghtB, True_LenghtA, True_LenghtB;
	double Dif_LenghtA = 0, Dif_LenghtB = 0;
	double Abs_Dif_LenghtA = 0, Abs_Dif_LenghtB = 0;
	int Sg_Dif_LenghtA = 0, Sg_Dif_LenghtB = 0;

	// Time
	double Time_Tot = 0, Time_For_Acc = 0, elapsed_time = 0;
	uint32_t start_cycles = 0, stop_cycles, cycles_elapsed;

	// Fan
	int Speed_Fan = 0;

	// Gyro/Acc
	float Angle_Acc_Roll, Angle_Acc_Pitch;
	float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
	float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
	double Cal_Gyro_Roll = 0, Cal_Gyro_Pitch = 0, Cal_Gyro_Yaw = 0;
	float Kalman1DOutput[] = {0, 0};

	// Coord
	float theta_polar = 0.0, phi_polar = 0.0;

	// Const
	float Pi = 3.141592653589793f;
	float Extradmot = 0.056f, Intradmot = 0.040f;
	float Extheigmot = 0.007f, Intheigmot = 0.099574f;
	float Thread_pitch = 0.0008f;
	short nb_step_motor = 200;
	float max_O_angle = 0.2118; // 0.2618
	float pitchRad;
	float rollRad;

	//MPU6050
	#define ACCEL_CONFIG_REG 0x1C
	#define GYRO_CONFIG_REG 0x1B
	float Ax, Ay, Az;
	float Gx, Gy, Gz;
	int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
	int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

	// Comunication
	uint8_t buf[100];
	int data=0;

	void MPU6050_Init(void) {
		uint8_t check, data;

		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
		if (check == 0x68){
			data = 0;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 1000);

			data = 0x07;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &data, 1, 1000);

			data = 0x00;//2g
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

			data = 0x08;//500°/s
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
		}
	}

	void MPU6050_Read_Sensors(void) {
		uint8_t Rec_Data[6];

		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 1000);
		Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
		Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
		Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

		Ax = (float)Accel_X_RAW / 16384.0 - 0.012;
		Ay = (float)Accel_Y_RAW / 16384.0 + 0.015;
		Az = (float)Accel_Z_RAW / 16384.0 - 0.08;

		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 1000);
		Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
		Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
		Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

		Gx = (float)Gyro_X_RAW / 65.5;
		Gy = (float)Gyro_Y_RAW / 65.5;
		Gz = (float)Gyro_Z_RAW / 65.5;
	}

	void DWT_Init(void) {
	  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	  DWT->CYCCNT = 0;
	  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
	void ReLenght(){
		CoordAP[0] = cos(270 * Pi / 180 - angleP) * Intradmot * cos(angleO) * cos(angleP) + sin(270 * Pi / 180 - angleP) * Intradmot * (-sin(angleP)) + Intheigmot * sin(angleO) * cos(angleP);
		CoordAP[1] = cos(270 * Pi / 180 - angleP) * Intradmot * cos(angleO) * sin(angleP) + sin(270 * Pi / 180 - angleP) * Intradmot * cos(angleP) + Intheigmot * sin(angleO) * sin(angleP);
		CoordAP[2] = cos(270 * Pi / 180 - angleP) * Intradmot * (-sin(angleO)) + Intheigmot * cos(angleO);

		CoordBP[0] = cos(30 * Pi / 180 - angleP) * Intradmot * cos(angleO) * cos(angleP) + sin(30 * Pi / 180 - angleP) * Intradmot * (-sin(angleP)) + Intheigmot * sin(angleO) * cos(angleP);
		CoordBP[1] = cos(30 * Pi / 180 - angleP) * Intradmot * cos(angleO) * sin(angleP) + sin(30 * Pi / 180 - angleP) * Intradmot * cos(angleP) + Intheigmot * sin(angleO) * sin(angleP);
		CoordBP[2] = cos(30 * Pi / 180 - angleP) * Intradmot * (-sin(angleO)) + Intheigmot * cos(angleO);

		LenghtA = sqrt((CoordA[0] - CoordAP[0]) * (CoordA[0] - CoordAP[0]) + (CoordA[1] - CoordAP[1]) * (CoordA[1] - CoordAP[1]) + (CoordA[2] - CoordAP[2]) * (CoordA[2] - CoordAP[2]));
		LenghtB = sqrt((CoordB[0] - CoordBP[0]) * (CoordB[0] - CoordBP[0]) + (CoordB[1] - CoordBP[1]) * (CoordB[1] - CoordBP[1]) + (CoordB[2] - CoordBP[2]) * (CoordB[2] - CoordBP[2]));

		Dif_LenghtA = LenghtA - True_LenghtA;
		Dif_LenghtB = LenghtB - True_LenghtB;

		if (Dif_LenghtA * Sg_Dif_LenghtA < 0 ||Dif_LenghtB * Sg_Dif_LenghtB < 0) {
			Current_Max_Freq = 0;
		}

	    Sg_Dif_LenghtA = (Dif_LenghtA < 0) ? -1 : 1;
	    Sg_Dif_LenghtB = (Dif_LenghtB < 0) ? -1 : 1;

		Abs_Dif_LenghtA = Sg_Dif_LenghtA * Dif_LenghtA;
		Abs_Dif_LenghtB = Sg_Dif_LenghtB * Dif_LenghtB;
	}
	void ReCoord(){
		CoordA[0] = cos(270 * Pi / 180) * Extradmot, CoordA[1] = sin(270 * Pi / 180) * Extradmot, CoordA[2] = Extheigmot;
		CoordB[0] = cos(30 * Pi / 180) * Extradmot, CoordB[1] = sin(30 * Pi / 180) * Extradmot, CoordB[2] = Extheigmot;
	}
	void Intitialisation(){
		ReCoord();
		angleP = 0;
		angleO = 0;
		ReLenght();
		True_LenghtA = LenghtA;
		True_LenghtB = LenghtB;
		sprintf((char*)buf,"\r\n \r\n");
		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
		TIM2->CCR1 = 30;
		TIM15->CCR2 = 30;
		DWT_Init();
		HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_buffer, 1);
		HAL_GPIO_WritePin(GPIOA, MotAR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, MotBR_Pin, GPIO_PIN_RESET);
		MPU6050_Init();
	}
	void ReTrueLenght_Speed() {
	    True_LenghtA += elapsed_time * Thread_pitch * Rota_SpeedA;
	    True_LenghtB += elapsed_time * Thread_pitch * Rota_SpeedB;

	    if (Abs_Dif_LenghtA > Precision_taille_verin || Abs_Dif_LenghtB > Precision_taille_verin) {
	        if (Abs_Dif_LenghtA > Abs_Dif_LenghtB) {
	            Presc_a = PWM_Freq / (Current_Max_Freq * PWM_Counter_Period) - 0.5;
	            Presc_b = PWM_Freq / ((Current_Max_Freq * PWM_Counter_Period) * (Abs_Dif_LenghtB / Abs_Dif_LenghtA)) - 0.5;

	            Rota_SpeedA = (Current_Max_Freq / nb_step_motor) * Sg_Dif_LenghtA;
	            Rota_SpeedB = (Current_Max_Freq * (Abs_Dif_LenghtB / Abs_Dif_LenghtA) / nb_step_motor) * Sg_Dif_LenghtB;
	        } else {
	            Presc_a = PWM_Freq / ((Current_Max_Freq * PWM_Counter_Period) * (Abs_Dif_LenghtA / Abs_Dif_LenghtB)) - 0.5;
	            Presc_b = PWM_Freq / (Current_Max_Freq * PWM_Counter_Period) - 0.5;

	            Rota_SpeedA = (Current_Max_Freq * (Abs_Dif_LenghtA / Abs_Dif_LenghtB) / nb_step_motor) * Sg_Dif_LenghtA;
	            Rota_SpeedB = (Current_Max_Freq / nb_step_motor) * Sg_Dif_LenghtB;
	        }

	        if (!HAL_GPIO_ReadPin(GPIOA, MotAR_Pin)) {
	            HAL_GPIO_WritePin(GPIOA, MotAR_Pin, GPIO_PIN_SET);
	            HAL_GPIO_WritePin(GPIOA, MotBR_Pin, GPIO_PIN_SET);
	        }
	    } else {
	        HAL_GPIO_WritePin(GPIOA, MotAR_Pin, GPIO_PIN_RESET);
	        HAL_GPIO_WritePin(GPIOA, MotBR_Pin, GPIO_PIN_RESET);

	        Rota_SpeedA = 0;
	        Rota_SpeedB = 0;
	        Current_Max_Freq = 0;
	    }

	    TIM2->ARR = Presc_b;
	    TIM15->ARR = Presc_a;

	    HAL_GPIO_WritePin(GPIOB, MotAD_Pin, (Dif_LenghtA < 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	    HAL_GPIO_WritePin(GPIOB, MotBD_Pin, (Dif_LenghtB < 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);

	    Time_For_Acc += elapsed_time;
	    Time_Tot += elapsed_time;
	}
	void ReAccel(){
		if (Current_Max_Freq == 0) Current_Max_Freq = Max_Start_Freq;
		else if (Time_For_Acc > Nb_Acc_p_action_accél / Max_Accel && Current_Max_Freq < Max_End_Freq){
			Current_Max_Freq = Current_Max_Freq + Nb_Acc_p_action_accél;
			Time_For_Acc = 0;
		}
	}
	void ReTime(){
		//Do not put anything between.
		stop_cycles = DWT->CYCCNT;
		cycles_elapsed = stop_cycles - start_cycles;
		start_cycles = DWT->CYCCNT;
		elapsed_time = (double)cycles_elapsed / HAL_RCC_GetHCLKFreq();
		ReTrueLenght_Speed();
		//Do not put anything between.
	}
	void CleanMess(){
	      for (int i = 0; i < BUFFER_SIZE; i++) {
	          if (rx_buffer[i] == '\r' || rx_buffer[i] == '\n') {
	              rx_buffer[i] = '\0';
	              break;
	          }
	      }

	    sprintf((char *)buf, "Rec: %s \r\n", rx_buffer);
        HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
	}
	void TestCode(){
        if (strncmp(rx_buffer, "Ang", 3) == 0 && sscanf(rx_buffer, "Ang:%f;%f", &angleO, &angleP) == 2) {
        }
        else if (strncmp(rx_buffer, "Fan", 3) == 0 && sscanf(rx_buffer, "Fan:%d", &Speed_Fan) == 1) {
    		TIM1->CCR4 = Speed_Fan;
		}
        else {
			   sprintf((char*)buf,"Unknown code\r\n");
			   HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
		}
	}

	void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
	    KalmanState += 0.004 * KalmanInput;
	    KalmanUncertainty += 0.06;//0.03
	    float KalmanGain = KalmanUncertainty / (KalmanUncertainty*KalmanUncertainty + 1.0);//0.9
	    KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);
	    KalmanUncertainty *= (1 - KalmanGain);
	    Kalman1DOutput[0] = KalmanState;
	    Kalman1DOutput[1] = KalmanUncertainty;
	}
	void Cal_Gyro(){
		Cal_Gyro_Roll = 0;
		Cal_Gyro_Pitch = 0;
		Cal_Gyro_Yaw = 0;
		for (int i = 0 ; i< 1000; i++) {
			MPU6050_Read_Sensors();
			Cal_Gyro_Roll+= Gx ;
		    Cal_Gyro_Pitch+= Gy ;
		    Cal_Gyro_Yaw+= Gz ;

		}
	    Cal_Gyro_Roll/= 1000 ;
	    Cal_Gyro_Pitch/= 1000 ;
	    Cal_Gyro_Yaw/= 1000 ;
	}
	void Re_Gyro_Angle() {
	    Gx -= Cal_Gyro_Roll;
	    Gy -= Cal_Gyro_Pitch;
	    Gz -= Cal_Gyro_Yaw;

	    // Calcul des angles d'accéléromètre
	    Angle_Acc_Roll = atan(Ay / sqrt(Ax * Ax + Az * Az)) * (180.0 / Pi);
	    Angle_Acc_Pitch = atan(Ax / sqrt(Ay * Ay + Az * Az)) * (180.0 / Pi);

	    // Application du filtre de Kalman pour les angles
	    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, Gx, Angle_Acc_Roll);
	    KalmanAngleRoll = Kalman1DOutput[0];
	    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

	    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, Gy, Angle_Acc_Pitch);
	    KalmanAnglePitch = Kalman1DOutput[0];
	    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

	    // Conversion des angles de Roll et Pitch en radians
	    rollRad = KalmanAngleRoll * M_PI / 180.0;
	    pitchRad = KalmanAnglePitch * M_PI / 180.0;

	    // Calcul des angles polaires
	    theta_polar = acos(cos(pitchRad) * cos(rollRad));  // Angle d’élévation
	    phi_polar = atan2(sin(rollRad), sin(pitchRad));    // Angle azimutal

	    if (phi_polar < 0) {
	            phi_polar += 2*Pi;
	    }

//		sprintf((char*)buf,"%.4f;%.4f;%.4f;%.4f;%.4f;%.4f\r\n",Ax,Ay,Az,Gx,Gy,Gz);
//		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
//		sprintf((char*)buf,"%.4f ; %.4f ; %.4f \r\n",Gx,Gy,Gz);
//		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
//		sprintf((char*)buf,"%.4f;%.4f\r\n",Angle_Acc_Roll,Angle_Acc_Pitch);
//		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
//		sprintf((char*)buf,"%.4f;%.4f;%.4f\r\n",Gx,Gy,Gz);
//		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
//		sprintf((char*)buf,"%.4f;%.4f;%.4f;%.4f\r\n",KalmanAngleRoll,KalmanAnglePitch,Angle_Acc_Roll,Angle_Acc_Pitch);
//		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
	}
	void Send_Data(){
		if (data>10) {
			sprintf((char*)buf,"%.4f;%.4f;%.4f\r\n",Time_Tot,rollRad,pitchRad);
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
			data=0;
		}
		else {
			data++;
		}
	}
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	TIM1->CCR4 = 100;  // Set the maximum pulse (2ms)
	HAL_Delay (3000);  // wait for 1 beep
	TIM1->CCR4 = 50;   // Set the minimum Pulse (1ms)
	HAL_Delay (2000);  // wait for 2 beeps
  Intitialisation();
  Cal_Gyro();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (message_ready) {
	      message_ready = false;
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
          CleanMess();
          TestCode();
      }

		ReLenght();
		ReAccel();
		ReTime();
		MPU6050_Read_Sensors();
		Re_Gyro_Angle();
		Send_Data();
		HAL_Delay(1);


//	    if (theta_polar > max_O_angle ) {
//	    	theta_polar = max_O_angle;
//		}
//
//	    angleO = theta_polar;
//	    angleP = phi_polar;


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1600-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 400;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 400;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 400;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MotAR_Pin|MotBR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MotBD_Pin|LD3_Pin|MotAD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MotAR_Pin MotBR_Pin */
  GPIO_InitStruct.Pin = MotAR_Pin|MotBR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MotBD_Pin LD3_Pin MotAD_Pin */
  GPIO_InitStruct.Pin = MotBD_Pin|LD3_Pin|MotAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {

        HAL_UART_Receive_IT(&huart2, (uint8_t *)&received_char, 1);

        if (received_char != '\n' && rx_index < BUFFER_SIZE - 1) {
            rx_buffer[rx_index++] = received_char;
        } else if (rx_index > 0) {
            rx_buffer[rx_index] = '\0';
            rx_index = 0;
            message_ready = true;
        }
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
  __disable_irq();
  while (1)
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
