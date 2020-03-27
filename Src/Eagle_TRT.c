#include "Eagle_TRT.h"
#include "stm32f7xx_hal_conf.h"

/*
 *Driver for all the stm in the Eagle_TRT veichle
 *incude this driver in the main file
 *you can't use all the functions unless you set up the CubeMx project correctly
*/

//----------------ENCODER----------------//
/*
 *To use encoder functions you have to initialize two timers, one for the clock and one to generate an interrupt
 *To get the rotational speed of the encoder, you can setup the interrupt timer and call 'encoder_tim_interrupt()';
 *you have to declare a variable and pass it as argument to the encoder_tim_interrupt() function.
 *The porpouse of that variable is to switch from the three phases needed to calculate the rotational speed.
 *The first and the second phases are to request two angles from the encoder, then third is to calculate the speed.
 *For the configuration of the first timer go to the description of the read_encoder() function.
 *For the configuration of the second timer you have to configure it to generate an interrupt.
 *That interrupt must be long enough to calculate a speed but not too long because you have to get the two angles in the same wheel rotation.
 *
 *working settings:
 *interrupt timer -> prescaler 36, counter period 1000
 *timer2 -> prescaler = 18, counter period = 65500
 *pin PC8 = data in
 *pin PC9 = clock pin
 *angles_array[15]
*/

//----------------IMU----------------//
/*
 *Setup SPI port to communicate with the sensor
 *Call gyro_init() and magn_accel_init() to setup the sensor
 *If you want to calibrate the sensor call gyro_calib() and accel_calib().
 *You can read the x, y, z data from the Gyroscope by calling gyro_read()
 *You can read the x, y, z data from the Gyroscope by calling accel_read()
*/

#ifdef HAL_SPI_MODULE_ENABLED
#include "stm32f4xx_hal_spi.h"

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

///IMU VARIABLES///
uint8_t ZERO = 0x00;
uint8_t WHO_AM_I_G = 0x8F;
uint8_t WHO_AM_I_G_VAL;
uint8_t WHO_AM_I_XM = 0x8F;
uint8_t WHO_AM_I_XM_VAL;

uint8_t CTRL_REG1_G_ADD = 0x20;
uint8_t CTRL_REG1_G_VAL = 0x0F;
uint8_t CTRL_REG4_G_ADD = 0x23;
uint8_t CTRL_REG4_G_VAL = 0x10;

uint8_t CTRL_REG1_XM_ADD = 0x20;
uint8_t CTRL_REG1_XM_VAL = 0xA7;
uint8_t CTRL_REG2_XM_ADD = 0x21;
uint8_t CTRL_REG2_XM_VAL = 0x08;
uint8_t CTRL_REG5_XM_ADD = 0x24;
uint8_t CTRL_REG5_XM_VAL = 0x70;
uint8_t CTRL_REG6_XM_ADD = 0x25;
uint8_t CTRL_REG6_XM_VAL = 0x20;
uint8_t CTRL_REG7_XM_ADD = 0x26;
uint8_t CTRL_REG7_XM_VAL = 0x00;

// GYROSCOPE SCALE SETTING
uint8_t CTRL_REG4_G = 0x23;

uint8_t SCL_G_245 = 0x00;
uint8_t SCL_G_500 = 0x10;
uint8_t SCL_G_1000 = 0x20;
uint8_t SCL_G_2000 = 0x30;

// ACCELERORMETER SCALE SETTING
uint8_t CTRL_REG2_XM = 0x21;

uint8_t SCL_A_2 = 0x00;
uint8_t SCL_A_4 = 0x08;
uint8_t SCL_A_6 = 0x10;
uint8_t SCL_A_8 = 0x18;
uint8_t SCL_A_16 = 0x20;

// MAGNETORMETER SCALE SETTING
uint8_t CTRL_REG6_XM = 0x25;

uint8_t SCL_M_2 = 0x00;
uint8_t SCL_M_4 = 0x20;
uint8_t SCL_M_8 = 0x40;
uint8_t SCL_M_12 = 0x60;

// OUTPUT REG
uint8_t OUT_X_L_G_ADD = 0xA8;
uint8_t OUT_X_H_G_ADD = 0xA9;
uint8_t OUT_Y_L_G_ADD = 0xAA;
uint8_t OUT_Y_H_G_ADD = 0xAB;
uint8_t OUT_Z_L_G_ADD = 0xAC;
uint8_t OUT_Z_H_G_ADD = 0xAD;

uint8_t OUT_X_L_A_ADD = 0xA8;
uint8_t OUT_X_H_A_ADD = 0xA9;
uint8_t OUT_Y_L_A_ADD = 0xAA;
uint8_t OUT_Y_H_A_ADD = 0xAB;
uint8_t OUT_Z_L_A_ADD = 0xAC;
uint8_t OUT_Z_H_A_ADD = 0xAD;

imu_stc accel;
imu_stc gyro;
can_stc can;

void send_config(imu_stc *imu, GPIO_TypeDef *pinx, uint16_t pinn, uint8_t *addr, uint8_t *val)
{
	HAL_GPIO_WritePin(pinx, pinn, GPIO_PIN_RESET); ///CS_InUse to 0
	htim2.Instance->CNT = 0;					   //set counter to 0
	while (htim2.Instance->CNT <= 20)
	{
	}										  //delay (must be >5ns)
	HAL_SPI_Transmit(imu->hspi, addr, 1, 10); ///Writing the address
	HAL_SPI_Transmit(imu->hspi, val, 1, 10);  ///Writing 0b00001111 to enable PowerMode and x,y,z axis
	htim2.Instance->CNT = 0;				  //set counter to 0
	while (htim2.Instance->CNT <= 20)
	{
	}											 //delay (must be >5ns)
	HAL_GPIO_WritePin(pinx, pinn, GPIO_PIN_SET); ///CS_InUse to 1
}

//accelerometer, gyroscope and magnetometer initialization
//call this function before requesting data from the sensor
//hspi = pointer to the spi port defined
void LSMD9S0_accel_gyro_init(imu_stc *accel, imu_stc *gyro)
{

	// Wake Up Gyro, enabling x, y, z axis
	send_config(gyro, gyro->GPIOx_InUse, gyro->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG1_G_ADD, (uint8_t *)&CTRL_REG1_G_VAL);

	HAL_Delay(1);

	// Wake Up Accel, enabling x, y, z axis
	send_config(accel, accel->GPIOx_InUse, accel->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG1_XM_ADD, (uint8_t *)&CTRL_REG1_XM_VAL);
	HAL_Delay(1);

	send_config(accel, accel->GPIOx_InUse, accel->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG5_XM_ADD, (uint8_t *)&CTRL_REG5_XM_VAL);
	HAL_Delay(1);
	send_config(accel, accel->GPIOx_InUse, accel->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG6_XM_ADD, (uint8_t *)&CTRL_REG6_XM_VAL);
	HAL_Delay(1);
	send_config(accel, accel->GPIOx_InUse, accel->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG7_XM_ADD, (uint8_t *)&CTRL_REG7_XM_VAL);
	HAL_Delay(1);

	// Set Gyro scale range
	switch (gyro->scale)
	{
	case 245:
		send_config(gyro, gyro->GPIOx_InUse, gyro->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG4_G, (uint8_t *)&SCL_G_245);
		break;
	case 500:
		send_config(gyro, gyro->GPIOx_InUse, gyro->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG4_G, (uint8_t *)&SCL_G_500);
		break;
	case 1000:
		send_config(gyro, gyro->GPIOx_InUse, gyro->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG4_G, (uint8_t *)&SCL_G_1000);
		break;
	case 2000:
		send_config(gyro, gyro->GPIOx_InUse, gyro->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG4_G, (uint8_t *)&SCL_G_2000);
		break;
	default:
		send_config(gyro, gyro->GPIOx_InUse, gyro->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG4_G, (uint8_t *)&SCL_G_245);
		gyro->scale = 500;
		break;
	}
	HAL_Delay(1);

	// Set Accel scale range
	switch (accel->scale)
	{
	case 2:
		send_config(accel, accel->GPIOx_InUse, accel->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG2_XM, (uint8_t *)&SCL_A_2);
		break;
	case 4:
		send_config(accel, accel->GPIOx_InUse, accel->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG2_XM, (uint8_t *)&SCL_A_4);
		break;
	case 6:
		send_config(accel, accel->GPIOx_InUse, accel->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG2_XM, (uint8_t *)&SCL_A_6);
		break;
	case 8:
		send_config(accel, accel->GPIOx_InUse, accel->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG2_XM, (uint8_t *)&SCL_A_8);
		break;
	case 16:
		send_config(accel, accel->GPIOx_InUse, accel->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG2_XM, (uint8_t *)&SCL_A_16);
		break;
	default:
		send_config(accel, accel->GPIOx_InUse, accel->GPIO_Pin_InUse, (uint8_t *)&CTRL_REG2_XM, (uint8_t *)&SCL_A_4);
		accel->scale = 4;
		break;
	}

	HAL_Delay(1);
	/*
	HAL_UART_Transmit(&huart2, (uint8_t *)"<IMU> Initialization -> Done\r\n", strlen("<IMU> Initialization -> Done\r\n"), 10);*/
}

float LSMD9S0_read(imu_stc *imu)
{

	uint8_t OUT_L_VAL;
	uint8_t OUT_H_VAL;

	HAL_GPIO_WritePin(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, GPIO_PIN_RESET); ///CS_InUse to 0
	htim2.Instance->CNT = 0;												  //set counter to 0
	while (htim2.Instance->CNT <= 20)
	{
	} //delay (must be >5ns)

	HAL_SPI_Transmit(imu->hspi, &(imu->REG_L), 1, 10);		  ///Writing LOW address
	HAL_SPI_Receive(imu->hspi, (uint8_t *)&OUT_L_VAL, 1, 10); ///Saving LOW data

	htim2.Instance->CNT = 0; //set counter to 0
	while (htim2.Instance->CNT <= 20)
	{
	}																		//delay (must be >5ns)
	HAL_GPIO_WritePin(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, GPIO_PIN_SET); ///CS_InUse to 1

	htim2.Instance->CNT = 0; //set counter to 0
	while (htim2.Instance->CNT <= 80)
	{
	}																		  //delay (must be >5ns)
	HAL_GPIO_WritePin(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, GPIO_PIN_RESET); ///CS_InUse to 0

	HAL_SPI_Transmit(imu->hspi, &(imu->REG_H), 1, 10);		  ///Writing HIGH address
	HAL_SPI_Receive(imu->hspi, (uint8_t *)&OUT_H_VAL, 1, 10); ///Saving HIGH data

	htim2.Instance->CNT = 0; //set counter to 0
	while (htim2.Instance->CNT <= 20)
	{
	} //delay (must be >5ns)

	HAL_GPIO_WritePin(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, GPIO_PIN_SET); ///CS_InUse to 1

	///CALCULATING ROTATION
	uint32_t value = (OUT_H_VAL << 8) | OUT_L_VAL; ///Calculating axis value shifting and using a logic OR
	float axis = value;

	return axis;
}

int LSMD9S0_check(imu_stc *imu)
{

	int check = 0;

	///AXEL/MAGN ARE WORKING
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
	htim2.Instance->CNT = 0;							  //set counter to 0
	while (htim2.Instance->CNT <= 20)
	{
	}															 //delay (must be >5ns)
	HAL_SPI_Transmit(imu->hspi, (uint8_t *)&WHO_AM_I_XM, 1, 10); ///Writing on register ----> (uint8_t*) it's the cast of the pointer to WHO_AM_I_G (giving by &variable)
	HAL_SPI_Receive(imu->hspi, (uint8_t *)&WHO_AM_I_XM_VAL, 1, 10);
	htim2.Instance->CNT = 0; //set counter to 0
	while (htim2.Instance->CNT <= 20)
	{
	}													//delay (must be >5ns)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

	///GYRO IS WORKING
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
	htim2.Instance->CNT = 0;							  //set counter to 0
	while (htim2.Instance->CNT <= 20)
	{
	}															//delay (must be >5ns)
	HAL_SPI_Transmit(imu->hspi, (uint8_t *)&WHO_AM_I_G, 1, 10); ///Writing on register ----> (uint8_t*) it's the cast of the pointer to WHO_AM_I_G (giving by &variable)
	//HAL_SPI_TransmitReceive(imu->hspi, (uint8_t*)&ZERO, (uint8_t*)&WHO_AM_I_G_VAL, 1, 10); ///Reading from register sending a 0x00
	HAL_SPI_Receive(imu->hspi, (uint8_t *)&WHO_AM_I_G_VAL, 1, 10);
	htim2.Instance->CNT = 0; //set counter to 0
	while (htim2.Instance->CNT <= 20)
	{
	}													//delay (must be >5ns)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1

	char txt[100];
	/*
	sprintf(txt, "Gyro, Accel check: %d\t%d\r\n", WHO_AM_I_G_VAL, WHO_AM_I_XM_VAL);
	HAL_UART_Transmit(&huart2, txt, strlen(txt), 10);*/

	///AXEL/GYRO STATUS
	if ((WHO_AM_I_G_VAL == 212) & (WHO_AM_I_XM_VAL == 73))
	{
		check = 1;
	}
	else
	{
		check = 0;
	}

	return check;
}

// Request inital axis values, average them, set as initial offsets
void LSM9DS0_calibration(imu_stc *imu)
{
	double x = 0, y = 0, z = 0;
	int iterations = 200;
	for (int i = 0; i < iterations; i++)
	{
		LSMD9S0_accel_read(imu);
		//imu_elaborate_data(imu);
		x += imu->x;
		y += imu->y;
		z += imu->z;
		HAL_Delay(1);
	}

	imu->x_offset = x / iterations;
	imu->y_offset = y / iterations;
	imu->z_offset = z / iterations;

	imu->calibration_done = 1;
}

//Reading G_axis values
//hspi = pointer to the spi port defined
//x = pointer gyroscope x variable
//y = pointer gyroscope y variable
//z = pointer gyroscope z variable
//x_offset = offset x value
//y_offset = offset y value
//z_offset = offset z value
void LSMD9S0_gyro_read(imu_stc *gyro)
{

	gyro->REG_H = OUT_X_H_G_ADD;
	gyro->REG_L = OUT_X_L_G_ADD;
	gyro->x = LSMD9S0_read(gyro);

	gyro->REG_H = OUT_Y_H_G_ADD;
	gyro->REG_L = OUT_Y_L_G_ADD;
	gyro->y = LSMD9S0_read(gyro);

	gyro->REG_H = OUT_Z_H_G_ADD;
	gyro->REG_L = OUT_Z_L_G_ADD;
	gyro->z = LSMD9S0_read(gyro);

	if (gyro->x > 32768)
	{
		gyro->x -= 65536;
	}
	if (gyro->y > 32768)
	{
		gyro->y -= 65536;
	}
	if (gyro->z > 32768)
	{
		gyro->z -= 65536;
	}

	gyro->x = gyro->x * ((float)gyro->scale / 32768);
	gyro->y = gyro->y * ((float)gyro->scale / 32768);
	gyro->z = gyro->z * ((float)gyro->scale / 32768);

	gyro->x -= gyro->x_offset;
	gyro->y -= gyro->y_offset;
	gyro->z -= gyro->z_offset;

	///AXEL/GYRO STATUS
	if ((WHO_AM_I_G_VAL == 212) & (WHO_AM_I_XM_VAL == 73))
	{
		gyro->error_flag = 0;
	}
	else
	{
		gyro->error_flag = 1;
	}
}

// Elaborating data
// Use the setted scale to calculate data in the correct ranges
// Aveage an array to remove noise
// Remove inital offset
void imu_elaborate_data(imu_stc *imu)
{
	if (imu->x > 32768)
	{
		imu->x -= 65536;
	}
	if (imu->y > 32768)
	{
		imu->y -= 65536;
	}
	if (imu->z > 32768)
	{
		imu->z -= 65536;
	}

	imu->x = imu->x * ((float)imu->scale / 32768);
	imu->y = imu->y * ((float)imu->scale / 32768);
	imu->z = imu->z * ((float)imu->scale / 32768);

	shift_array(imu->x_array, 10, imu->x);
	shift_array(imu->y_array, 10, imu->y);
	shift_array(imu->z_array, 10, imu->z);

	imu->x = dynamic_average(imu->x_array, 10);
	imu->y = dynamic_average(imu->y_array, 10);
	imu->z = dynamic_average(imu->z_array, 10);

	imu->x -= imu->x_offset;
	imu->y -= imu->y_offset;
	imu->z -= imu->z_offset;
}

///Reading A_axis values
//hspi = pointer to the spi port defined
//x = pointer gyroscope x variable
//y = pointer gyroscope y variable
//z = pointer gyroscope z variable
//x_offset = offset x value
//y_offset = offset y value
//z_offset = offset z value
void LSMD9S0_accel_read(imu_stc *accel)
{

	accel->REG_H = OUT_X_H_A_ADD;
	accel->REG_L = OUT_X_L_A_ADD;
	accel->x = LSMD9S0_read(accel);

	accel->REG_H = OUT_Y_H_A_ADD;
	accel->REG_L = OUT_Y_L_A_ADD;
	accel->y = LSMD9S0_read(accel);

	accel->REG_H = OUT_Z_H_A_ADD;
	accel->REG_L = OUT_Z_L_A_ADD;
	accel->z = LSMD9S0_read(accel);

	if (accel->x > 32768)
	{
		accel->x -= 65536;
	}
	if (accel->y > 32768)
	{
		accel->y -= 65536;
	}
	if (accel->z > 32768)
	{
		accel->z -= 65536;
	}

	accel->x = accel->x * ((float)accel->scale / 32768);
	accel->y = accel->y * ((float)accel->scale / 32768);
	accel->z = accel->z * ((float)accel->scale / 32768);

	accel->x -= accel->x_offset;
	accel->y -= accel->y_offset;
	accel->z -= accel->z_offset;
}

#endif

#ifdef HAL_CAN_MODULE_ENABLED
#include "stm32f4xx_hal_can.h"
//function that sends an array via CAN
//hcan = pointer to can port
//id = id of the message to be sent
//dataTx = pointer to array that contains the data to be sent
//size = size of the array
can_stc can;
int CAN_Send(can_stc *can)
{

	uint32_t mailbox;
	uint8_t flag = 0;

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = can->id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = can->size;
	TxHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_GetTxMailboxesFreeLevel(can->hcan) != 0 && HAL_CAN_IsTxMessagePending(can->hcan, CAN_TX_MAILBOX0) == 0)
	{
		HAL_CAN_AddTxMessage(can->hcan, &TxHeader, can->dataTx, &mailbox);
		flag = 1;
	}

	return flag;
}

//receive a buffer from the CAN communication
//you can call this function in the callback of the CAN interrupt
//hcan = pointer to can port
//DataRx = pointer to the buffer you are receiveng
//size = size of the buffer you are using
int CAN_Receive(can_stc *can)
{

	CAN_RxHeaderTypeDef RxHeader;

	if (HAL_CAN_GetRxFifoFillLevel(can->hcan, CAN_RX_FIFO0) != 0)
	{
		HAL_CAN_GetRxMessage(can->hcan, CAN_RX_FIFO0, &RxHeader, can->dataRx);
	}

	int id = RxHeader.StdId;

	return id;
}
#endif

#ifdef HAL_UART_MODULE_ENABLED
#include "stm32f7xx_hal_uart.h"
#include "malloc.h"
///---- queue ---- ///
static int next(int ret, int dim)
{

	return (ret + 1) % dim;
}

// Implementazione dinamica
void init(queue *q)
{

	q->tail = q->head = 0;
	q->dim = 40;
}

static int emptyp(const queue *q)
{

	return (q->tail == q->head);
}

static int fullp(const queue *q)
{

	return (next(q->tail, q->dim) == q->head);
}

int push(char *str, queue *q)
{
	int res;
	if (fullp(q))
	{
		res = FAIL;
	}
	else
	{
		int length = strlen(str);
		q->elem[q->tail] = (char *)malloc(sizeof(char) * length);
		strcpy(q->elem[q->tail], str);
		//q->tail = next(q->tail,q->dim);
		if (q->tail == 39)
		{
			q->tail = 0;
		}
		else
		{
			q->tail++;
		}
		res = OK;
	}

	return res;
}

int pop(char *str, queue *q)
{
	int res;
	if (emptyp(q))
	{
		res = FAIL;
	}
	else
	{
		strcpy(str, q->elem[q->head]);
		free(q->elem[q->head]);
		//q->head = next(q->head,q->dim);
		if (q->head == 39)
		{
			q->head = 0;
		}
		else
		{
			q->head++;
		}
		res = OK;
	}

	return res;
}
/// ---- end queue ----///
queue print_q = {.head = 0, .tail = 0, .dim = 0};

int print(UART_HandleTypeDef *huart, char *text_print_function)
{

	int ret = 0;

	if (HAL_UART_Transmit_IT(huart, (uint8_t *)text_print_function, strlen(text_print_function)) == HAL_OK)
	{
		ret = 1;
	}
	else
	{
		if (push(text_print_function, &print_q) == FAIL)
		{
			ret = 0;
		}
		else
		{
			ret = 1;
		}
	}

	return ret;
}

void print_it(UART_HandleTypeDef *huart)
{ //put in the uart interrupt

	char text_print_function[50];

	if (pop(text_print_function, &print_q) == OK)
	{
		HAL_UART_Transmit_IT(huart, (uint8_t *)text_print_function, strlen(text_print_function));
	}
}

#endif

#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f7xx_hal_tim.h"

extern UART_HandleTypeDef huart2;
extern char txt;

// Function to request data from encoder via SSI communication
// This function is called from the interrupt callback of the timer that you are using for the encoder
// The tim used for this function must be initialized at most at 2 microsecond per tick
// Lower the number of microseconds per tick better it is
// TimerInstance = struct of the tim used for the encoder
enc_stc enc;
int read_SSI(enc_stc *enc)
{

	int bin_data[enc->data_size];

	HAL_GPIO_WritePin(enc->ClockPinName, enc->ClockPinNumber, GPIO_PIN_RESET);
	__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
	while (__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period)
	{
	}
	HAL_GPIO_WritePin(enc->ClockPinName, enc->ClockPinNumber, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
	while (__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period)
	{
	}
	HAL_GPIO_WritePin(enc->ClockPinName, enc->ClockPinNumber, GPIO_PIN_RESET);
	__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
	while (__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period)
	{
	}

	// Starting the clock to retrieve 14 bits from the sensor
	for (int i = 0; i < enc->data_size; i++)
	{

		// CLOCK HIGH
		HAL_GPIO_WritePin(enc->ClockPinName, enc->ClockPinNumber, GPIO_PIN_SET);
		__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
		while (__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period / 2)
		{
		}

		//Reading the Pin at the half of the clock period
		// Set the bit as the pin state (0 or 1)
		bin_data[i] = HAL_GPIO_ReadPin(enc->DataPinName, enc->DataPinNumber);
		enc->Data[i] = bin_data[i];

		__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
		while (__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period / 2)
		{
		}

		// CLOCK LOW
		HAL_GPIO_WritePin(enc->ClockPinName, enc->ClockPinNumber, GPIO_PIN_RESET);
		__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
		while (__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period)
		{
		}
	}

	// Requesting an other bit for the aventual error sent from the sensor
	HAL_GPIO_WritePin(enc->ClockPinName, enc->ClockPinNumber, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
	while (__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period * 2)
	{
	}

	int error_flag = HAL_GPIO_ReadPin(enc->DataPinName, enc->DataPinNumber);

	__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
	while (__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period * 2)
	{
	}

	// Converting bits into number and converting it into angle in degrees (0 ~ 359)
	enc->converted_data = bin_dec(enc->Data, enc->data_size);

	return error_flag;
}

// Interrupt function of tim 2
// Call this function in the timer callback function of the stm
// htim = timer TimerInstance of the timer that you are using for the clock of the encoder
// Interrupt_flag = initilize a int variable in the main file
// Angles_array = array to store the last angles
// Speed = pointer to the speed value
void encoder_tim_interrupt(enc_stc *enc)
{

	if (enc->interrupt_flag == 0)
	{
		// Requesting first angle
		enc->angle0_prec = enc->angle0;
		read_SSI(enc);
		enc->angle0 = enc->converted_data / 45.5055;
	}
	else if (enc->interrupt_flag == 1)
	{
		// Requesting second angle
		enc->angle1_prec = enc->angle1;
		read_SSI(enc);
		enc->angle1 = enc->converted_data / 45.5055;
	}
	else if (enc->interrupt_flag == 2)
	{
		// Calculate speed from the two angles
		get_speed_encoder(enc);

		// Get the speed sign to be sent in CAN
		if (enc->average_speed < 0)
		{
			enc->average_speed *= -1;
			enc->speed_sign = 1;
		}
		else
		{
			enc->speed_sign = 0;
		}
	}

	// Cycle between steps
	if (enc->interrupt_flag >= 2)
	{
		enc->interrupt_flag = 0;
	}
	else
	{
		enc->interrupt_flag++;
	}
	//enc->interrupt_flag = enc->interrupt_flag >= 2 ? 0 : enc->interrupt_flag + 1;
}

// Funtion to calculate the speed
// Angle0 = last angle calculated
// Angle1 = previous angle calculated
// Refresh = delta-time from the two calculations, express it in microseconds
// Wheel_diameter = diameter of the wheel expressed meters
void get_speed_encoder(enc_stc *enc)
{

	long double speed = 0;
	double dt = 0;

	if (enc->dx_wheel == 1)
	{
		enc->delta_angle = enc->angle1 - enc->angle0;
	}
	else
	{
		enc->delta_angle = enc->angle0 - enc->angle1;
	}

	// Calculate correct delta angle if near to 0-360
	if ((enc->angle0 < enc->max_delta_angle && enc->angle1 > 360 - enc->max_delta_angle) ||
		(enc->angle1 < enc->max_delta_angle && enc->angle0 > 360 - enc->max_delta_angle))
	{
		if (enc->delta_angle < 0)
		{
			enc->delta_angle = 360 + enc->delta_angle;
		}
		else
		{
			if (enc->delta_angle > 0)
			{
				enc->delta_angle = 360 - enc->delta_angle;
			}
		}
	}

	// Calculating rad/s, then m/s, then Km/h
	//speed = enc->delta_angle * (3.1415 / 180) * (enc->wheel_diameter / 2);
	speed = (enc->delta_angle / 360) * ((enc->wheel_diameter / 2) * 3.1415);
	speed *= enc->frequency;
	speed *= 3.6;
	speed = round((speed * 1000)) / 1000;

	int off = 100;

	// Start detecting eventual new wheel roation
	// If the speed is too low, don't count rotations
	if (enc->average_speed < -0.5 || enc->average_speed > 0.5)
	{
		if ((enc->angle0_prec <= 361 && enc->angle0_prec > 360 - off) && (enc->angle0 >= -1 && enc->angle0 < off) ||
			(enc->angle0_prec >= -1 && enc->angle0_prec < off) && (enc->angle0 <= 361 && enc->angle0 > 360 - off))
		{
			enc->wheel_rotation++;
			enc->Km += (3.14 * enc->wheel_diameter);
		}
	}

	// Remove noise mediating previous values with actual
	shift_array(enc->speed_array, 4, speed);
	enc->average_speed = dynamic_average(enc->speed_array, 4);

	// Calculating the angle sample frequency
	//enc_calculate_optimal_frequency(enc);
}
/*
// Calculate anche sample frequency
// The delta angle changes depending on the current speed
// Constrain the delta angle between a defined range (max_delta_angle)
void enc_calculate_optimal_frequency(enc_stc *enc)
{
	double abs_delta_angle = (enc->delta_angle >= 0) ? enc->delta_angle : enc->delta_angle * -1;

	if (abs_delta_angle > enc->max_delta_angle * 1.2 || abs_delta_angle < enc->max_delta_angle * 0.5)
	{
		double angular_speed = enc->average_speed / ((enc->wheel_diameter / 2)*3.6);
		double frequency = angular_speed / (enc->max_delta_angle * 3.14 / 180);

		enc->flake_freq = frequency;

		frequency = frequency < 0 ? frequency * (-1) : frequency;
		frequency = frequency < 50 ? 50 : frequency;

		// Returns 0 if reinitialization is done correctly
		HAL_TIM_Base_Stop(enc->frequency_timer);
		HAL_TIM_Base_Stop_IT(enc->frequency_timer);
		if (ReinitTIM7(frequency, enc) == 0)
		{
			// Set encoder actual frequency
			enc->frequency = enc->frequency_timer_Hz / (enc->frequency_timer->Init.Prescaler * enc->frequency_timer->Init.Period);
		}
		HAL_TIM_Base_Start(enc->frequency_timer);
		HAL_TIM_Base_Start_IT(enc->frequency_timer);
	}
}

// Reinitializing timer to generate interrupts to the given frequency
int ReinitTIM7(float frequency, enc_stc *enc)
{

	int error_flag = 0;


	TIM_MasterConfigTypeDef sMasterConfig = {0};


	enc->frequency_timer_prescaler = sqrt(enc->frequency_timer_Hz / frequency);
	enc->frequency_timer_period = enc->frequency_timer_prescaler;

	enc->frequency_timer->Instance = TIM7;
	enc->frequency_timer->Init.Prescaler = enc->frequency_timer_prescaler;
	enc->frequency_timer->Init.CounterMode = TIM_COUNTERMODE_UP;
	enc->frequency_timer->Init.Period = enc->frequency_timer_period;
	enc->frequency_timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(enc->frequency_timer) != HAL_OK)
	{
		error_flag = 1;
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(enc->frequency_timer, &sMasterConfig) != HAL_OK)
	{
		error_flag = 2;
	}
	return error_flag;
}*/

pot_stc pot_1;
pot_stc pot_2;
pot_stc pot_3;
void calc_pot_value(pot_stc *pot)
{

	pot->val_100 = round(100 - (abs(pot->val - pot->min) * 100 / (pot->range))); //val0_100 -->STEER --> 0 = SX | 100 = DX
	if (pot->val <= pot->min)
	{
		pot->val_100 = 100;
	}
	if (pot->val >= pot->max)
	{
		pot->val_100 = 0;
	}
}

//Function to check if the two ADC values are approximately the same
//if the values are different for more tha 10 points percentage for more than 100 milliseconds returns the SCS Error
//TimerInstance = pointer to the timer needed to check the SCS error
//val0_100 = pointer to the first potentiometer
//val1_100 = pointer to the second potentiometer
int implausibility_check(pot_stc *pot_1, pot_stc *pot_2)
{

	int SCS1 = 0;

	if (pot_1->val_100 >= 100)
	{
		pot_1->val_100 = 100;
	}
	if (pot_1->val_100 <= 5)
	{
		pot_1->val_100 = 0;
	}
	if (pot_2->val_100 >= 100)
	{
		pot_2->val_100 = 100;
	}
	if (pot_2->val_100 <= 5)
	{
		pot_2->val_100 = 0;
	}
	if (abs(pot_1->val_100 - pot_2->val_100) >= 10)
	{
		if (__HAL_TIM_GET_COUNTER(pot_1->TimerInstance) > 100)
		{
			SCS1 = 1;
		}
	}
	else
	{
		__HAL_TIM_SET_COUNTER(pot_1->TimerInstance, 0);
		SCS1 = 0;
	}

	return SCS1;
}

//function to set the value of the potentiometer when the pedal is released
//val = array pointer to the potentiometer values
//max1 = pointer to the maximum value of the APPS1
//max2 = pointer to the maximum value of the APPS2
void set_max(pot_stc *pot_1)
{
	pot_1->max = pot_1->val;
}

//function to set the value of the potentiometer when the pedal is pressed
//val = array pointer to the potentiometer values
//min1 = pointer to the minimum value of the APPS1
//min2 = pointer to the minimum value of the APPS2
void set_min(pot_stc *pot_1)
{
	pot_1->min = pot_1->val;
}

#endif

//function to calculate the decimal value from MSB binary array
//bin = pointer to binary array
//max = size of the array
int bin_dec(int *bin, int size)
{

	int dec = 0;

	for (int i = 0; i < size; i++)
	{
		if (bin[i] == 1)
		{
			dec += Power(2, size - i - 1);
		}
	}
	return dec;
}

//function to calculate the power of a given number
double Power(int base, int expn)
{

	double result = 1;

	if (expn != 0)
	{
		for (int j = 0; j < expn; j++)
		{
			result = result * base;
		}
	}

	return result;
}

//shift all the data of a numeric array and add another one value
//array = array to be shifted
//size = size of the array
//data = value to be added in the last position of the array
void shift_array(long double *array, int size, long double data)
{

	for (int i = 1; i < size; i++)
	{
		array[i - 1] = array[i];
	}
	array[size - 1] = data;
}

double speed_filter(double *data, int size)
{
	double min = 100000000000000;
	double max = -min;
	double sum = 0;
	double average = 0;
	int index_1;
	int index_2;
	int average_members = 0;

	for (int i = 0; i < size; i++)
	{
		if (data[i] < min)
		{
			min = data[i];
			index_1 = i;
		}

		if (data[i] > max)
		{
			max = data[i];
			index_2 = i;
		}
	}

	for (int i = 0; i < size; i++)
	{
		if (i != index_1 || i != index_2)
		{
			sum += data[i];
			average_members++;
		}
	}
	average = sum / average_members;

	return average;
}

//function that calculate the average of all the numbers in one array
double dynamic_average(long double *array, int size)
{

	double sum = 0;
	double average = 0;

	for (int i = 0; i < size; i++)
	{
		sum += array[i];
	}
	average = sum / size;

	return average;
}
