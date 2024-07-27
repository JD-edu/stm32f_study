#include "bsp.h"
#include "math.h"
#include "stdlib.h"


// The LED displays the current operating status, which is invoked every 10 milliseconds, and the LED blinks every 200 milliseconds.  
void Bsp_Led_Show_State_Handle(void)
{
	static uint8_t led_count = 0;
	led_count++;
	if (led_count > 20)
	{
		led_count = 0;
//		LED_TOGGLE();
	}
}


// The peripheral device is initialized
void Bsp_Init(void)
{
	USART1_Init();
	ICM20948_init();
	AK09916_init();
	Encoder_Init();
	PwmServo_Init();
	Beep_On_Time(50);
}
//float pulse = 0;
extern float spin;
extern float setpoint;
extern float setvel;
extern UART_HandleTypeDef huart1;
extern uint8_t RxTemp;
extern uint8_t RxComplete;
extern int m_speed;
extern int g_Encoder_M1_Now;
extern int g_Encoder_M2_Now;

int error_speed = 0;
int sit_count=0;
extern int setpoint_encoder;
extern float speed;
extern float speed_e_pid;
extern float min_setpoint;
extern float max_setpoint;
extern int set_angle;
int jumpspeed=2;
//
//int encoder[2] = {0};
//int show_encoder = 0;

//int angle1 = 90;
//int angle2 = 90;

int state = 0;
int jump = 0;
// main.c。
// This function is called in a loop in main.c to avoid multiple modifications to the main.c file
void Bsp_Loop(void)
{



	HAL_UART_RxCpltCallback(&huart1);

	if(RxComplete)
	{
		if(RxTemp == '0') //start
				{
					state=1;

				}
		if(RxTemp == '1') //end
				{
					state=0;
					Motor_Set_Pwm(MOTOR_ID_M1, 0);
					Motor_Set_Pwm(MOTOR_ID_M2, 0);

				}

		if(RxTemp == '6') //forward
				{
					setvel=50;

				}
		if(RxTemp == '7') //backward
		{
			setvel=-50;
		}
		if(RxTemp == '5') //stop
		{
			setvel=0;
		}
		if(RxTemp == '8')//left
		{
			setvel=0;
			for(int i=1;i<300;i++){
			setpoint_encoder+=3;
			HAL_Delay(2);
			}
		}
		if(RxTemp == '9')//right
		{
			setvel=0;
			for(int i=1;i<300;i++){
			setpoint_encoder-=3;
			HAL_Delay(2);
			}
		}


	}

	RxComplete = 0;
	HAL_UART_Receive_IT(&huart1, &RxTemp, 1);

	if (Key1_State(KEY_MODE_ONE_TIME))
	{

		state++;
	}
	if(state==2){
				setvel=30;
				HAL_Delay(2000);
				leftdown(30);
//				for(int i=1;i<2000;i++){
//				setpoint_encoder+=3;
//				HAL_Delay(2);
//				}
//				leftup(30);

		}
	else if(state==3){
		for(int i=1;i<1000;i++){
		setpoint_encoder-=10;
		HAL_Delay(2);
		}
		state++;
		}
	else if(state==4){
		setvel=0;
		//HAL_Delay(5000);
		for(int i=1;i<1000;i++){
		setpoint_encoder+=10;
		HAL_Delay(2);
		}
		state=3;
	}



	Bsp_Led_Show_State_Handle();
	Beep_Timeout_Close_Handle();
	HAL_Delay(10);
}

void sitdown(int sit_speed){
	for(int i=1;i<41;i++){
		PwmServo_Set_Angle_All(40-i, 140+i, 30, 30);
		min_setpoint-=0.308;
		max_setpoint-=0.308;
		setpoint-=0.308;
		HAL_Delay(sit_speed);
	}
}
void standup(int sit_speed){
	for(int i=1;i<41;i++){
		PwmServo_Set_Angle_All(i, 180-i, 30, 30);
		min_setpoint+=0.308;
		max_setpoint+=0.308;
		setpoint+=0.308;
		HAL_Delay(sit_speed);
	}
}
void leftdown(int sit_speed){
	for(int i=1;i<41;i++){
		PwmServo_Set_Angle_All(40, 140+i, 30, 30);
		min_setpoint-=0.184;
		max_setpoint-=0.184;
		setpoint-=0.184;
		HAL_Delay(sit_speed);
	}
}
void leftup(int sit_speed){
	for(int i=1;i<41;i++){
		PwmServo_Set_Angle_All(40, 180-i, 30, 30);
		min_setpoint+=0.184;
		max_setpoint+=0.184;
		setpoint+=0.184;
		HAL_Delay(sit_speed);
	}
}
void rightdown(int sit_speed){
	for(int i=1;i<41;i++){
		PwmServo_Set_Angle_All(40-i,140, 30, 30);
		min_setpoint-=0.184;
		max_setpoint-=0.184;
		setpoint-=0.184;
		HAL_Delay(sit_speed);
	}
}
void rightup(int sit_speed){
	for(int i=1;i<41;i++){
		PwmServo_Set_Angle_All(i, 140, 30, 30);
		min_setpoint+=0.184;
		max_setpoint+=0.184;
		setpoint+=0.184;
		HAL_Delay(sit_speed);
	}
}
