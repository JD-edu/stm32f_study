#include "bsp.h"
#include "math.h"
#include "bsp_icm20948.h"
#include "tim.h"

static float g_scale_gyro = 1;
static float g_scale_accel = 1;

axises_t g_axises_gyro;
axises_t g_axises_accel;
axises_t g_axises_mag;
raw_data_t g_raw_gyro;
raw_data_t g_raw_accel;
raw_data_t g_raw_mag;
uint16_t print_count = 0;
int m_speed=0;


static void ICM20948_NoActive()
{
	HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, SET);
}

static void ICM20948_Active()
{
	HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, RESET);
}

static void select_user_bank(userbank_t ub)
{
	uint8_t write_reg[2];
	write_reg[0] = WRITE | REG_BANK_SEL;
	write_reg[1] = ub;

	ICM20948_Active();
	HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 10);
	ICM20948_NoActive();
}

static uint8_t read_single_reg(userbank_t ub, uint8_t reg)
{
	uint8_t read_reg = READ | reg;
	uint8_t reg_val;
	select_user_bank(ub);

	ICM20948_Active();
	HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
	HAL_SPI_Receive(ICM20948_SPI, &reg_val, 1, 1000);
	ICM20948_NoActive();
	return reg_val;
}

static void write_single_reg(userbank_t ub, uint8_t reg, uint8_t val)
{
	uint8_t write_reg[2];
	write_reg[0] = WRITE | reg;
	write_reg[1] = val;

	select_user_bank(ub);

	ICM20948_Active();
	HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 1000);
	ICM20948_NoActive();
}

static uint8_t* read_multiple_reg(userbank_t ub, uint8_t reg, uint8_t len)
{
	uint8_t read_reg = READ | reg;
	static uint8_t reg_val[6];
	select_user_bank(ub);

	ICM20948_Active();
	HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
	HAL_SPI_Receive(ICM20948_SPI, reg_val, len, 1000);
	ICM20948_NoActive();

	return reg_val;
}

static void write_multiple_reg(userbank_t ub, uint8_t reg, uint8_t* val, uint8_t len)
{
	uint8_t write_reg = WRITE | reg;
	select_user_bank(ub);

	ICM20948_Active();
	HAL_SPI_Transmit(ICM20948_SPI, &write_reg, 1, 1000);
	HAL_SPI_Transmit(ICM20948_SPI, val, len, 1000);
	ICM20948_NoActive();
}

static uint8_t read_single_mag_reg(uint8_t reg)
{
	write_single_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);

	HAL_Delay(1);
	return read_single_reg(ub_0, B0_EXT_SLV_SENS_DATA_00);
}

static void write_single_mag_reg(uint8_t reg, uint8_t val)
{
	write_single_reg(ub_3, B3_I2C_SLV0_ADDR, WRITE | MAG_SLAVE_ADDR);
	write_single_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_reg(ub_3, B3_I2C_SLV0_DO, val);
	write_single_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);
}

static uint8_t* read_multiple_mag_reg(uint8_t reg, uint8_t len)
{
	write_single_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_reg(ub_3, B3_I2C_SLV0_CTRL, 0x80 | len);

	HAL_Delay(1);
	return read_multiple_reg(ub_0, B0_EXT_SLV_SENS_DATA_00, len);
}


static void ICM20948_device_reset()
{
	write_single_reg(ub_0, B0_PWR_MGMT_1, 0x80 | 0x41);
	HAL_Delay(100);
}

static void AK09916_soft_reset()
{
	write_single_mag_reg(MAG_CNTL3, 0x01);
	HAL_Delay(100);
}

static void ICM20948_wakeup()
{
	uint8_t new_val = read_single_reg(ub_0, B0_PWR_MGMT_1);
	new_val &= 0xBF;

	write_single_reg(ub_0, B0_PWR_MGMT_1, new_val);
	HAL_Delay(100);
}

static void ICM20948_spi_slave_enable()
{
	uint8_t new_val = read_single_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x10;

	write_single_reg(ub_0, B0_USER_CTRL, new_val);
}

static void ICM20948_i2c_master_reset()
{
	uint8_t new_val = read_single_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x02;

	write_single_reg(ub_0, B0_USER_CTRL, new_val);
}

static void ICM20948_i2c_master_enable()
{
	uint8_t new_val = read_single_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x20;

	write_single_reg(ub_0, B0_USER_CTRL, new_val);
	HAL_Delay(100);
}

static void ICM20948_i2c_master_clk_frq(uint8_t config)
{
	uint8_t new_val = read_single_reg(ub_3, B3_I2C_MST_CTRL);
	new_val |= config;

	write_single_reg(ub_3, B3_I2C_MST_CTRL, new_val);
}

static void ICM20948_clock_source(uint8_t source)
{
	uint8_t new_val = read_single_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= source;

	write_single_reg(ub_0, B0_PWR_MGMT_1, new_val);
}

static void ICM20948_odr_align_enable()
{
	write_single_reg(ub_2, B2_ODR_ALIGN_EN, 0x01);
}

static void ICM20948_gyro_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_reg(ub_2, B2_GYRO_CONFIG_1);
	new_val |= config << 3;

	write_single_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

static void ICM20948_accel_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_reg(ub_2, B2_ACCEL_CONFIG);
	new_val |= config << 3;

	write_single_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

static void ICM20948_gyro_sample_rate_divider(uint8_t divider)
{
	write_single_reg(ub_2, B2_GYRO_SMPLRT_DIV, divider);
}

static void ICM20948_accel_sample_rate_divider(uint16_t divider)
{
	uint8_t divider_1 = (uint8_t)(divider >> 8);
	uint8_t divider_2 = (uint8_t)(0x0F & divider);

	write_single_reg(ub_2, B2_ACCEL_SMPLRT_DIV_1, divider_1);
	write_single_reg(ub_2, B2_ACCEL_SMPLRT_DIV_2, divider_2);
}

static void AK09916_operation_mode_setting(operation_mode_t mode)
{
	write_single_mag_reg(MAG_CNTL2, mode);
	HAL_Delay(100);
}

static void ICM20948_gyro_calibration()
{
	raw_data_t temp;
	int32_t gyro_bias[3] = {0};
	uint8_t gyro_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		ICM20948_gyro_read(&temp);
		gyro_bias[0] += temp.x;
		gyro_bias[1] += temp.y;
		gyro_bias[2] += temp.z;
	}

	gyro_bias[0] /= 100;
	gyro_bias[1] /= 100;
	gyro_bias[2] /= 100;

	// Construct the gyro biases for push to the hardware gyro bias registers,
	// which are reset to zero upon device startup.
	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
	// Biases are additive, so change sign on calculated average gyro biases
	gyro_offset[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF;
	gyro_offset[1] = (-gyro_bias[0] / 4)       & 0xFF;
	gyro_offset[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	gyro_offset[3] = (-gyro_bias[1] / 4)       & 0xFF;
	gyro_offset[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	gyro_offset[5] = (-gyro_bias[2] / 4)       & 0xFF;

	write_multiple_reg(ub_2, B2_XG_OFFS_USRH, gyro_offset, 6);
}

static void ICM20948_accel_calibration()
{
	raw_data_t temp;
	uint8_t* temp2;
	uint8_t* temp3;
	uint8_t* temp4;

	int32_t accel_bias[3] = {0};
	int32_t accel_bias_reg[3] = {0};
	uint8_t accel_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		ICM20948_accel_read(&temp);
		accel_bias[0] += temp.x;
		accel_bias[1] += temp.y;
		accel_bias[2] += temp.z;
	}

	accel_bias[0] /= 100;
	accel_bias[1] /= 100;
	accel_bias[2] /= 100;

	uint8_t mask_bit[3] = {0, 0, 0};

	temp2 = read_multiple_reg(ub_1, B1_XA_OFFS_H, 2);
	accel_bias_reg[0] = (int32_t)(temp2[0] << 8 | temp2[1]);
	mask_bit[0] = temp2[1] & 0x01;

	temp3 = read_multiple_reg(ub_1, B1_YA_OFFS_H, 2);
	accel_bias_reg[1] = (int32_t)(temp3[0] << 8 | temp3[1]);
	mask_bit[1] = temp3[1] & 0x01;

	temp4 = read_multiple_reg(ub_1, B1_ZA_OFFS_H, 2);
	accel_bias_reg[2] = (int32_t)(temp4[0] << 8 | temp4[1]);
	mask_bit[2] = temp4[1] & 0x01;

	accel_bias_reg[0] -= (accel_bias[0] / 8);
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  	accel_offset[1] = (accel_bias_reg[0])      & 0xFE;
	accel_offset[1] = accel_offset[1] | mask_bit[0];

	accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  	accel_offset[3] = (accel_bias_reg[1])      & 0xFE;
	accel_offset[3] = accel_offset[3] | mask_bit[1];

	accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	accel_offset[5] = (accel_bias_reg[2])      & 0xFE;
	accel_offset[5] = accel_offset[5] | mask_bit[2];

	write_multiple_reg(ub_1, B1_XA_OFFS_H, &accel_offset[0], 2);
	write_multiple_reg(ub_1, B1_YA_OFFS_H, &accel_offset[2], 2);
	write_multiple_reg(ub_1, B1_ZA_OFFS_H, &accel_offset[4], 2);
}

static void ICM20948_gyro_full_scale_select(gyro_scale_t full_scale)
{
	uint8_t new_val = read_single_reg(ub_2, B2_GYRO_CONFIG_1);

	switch(full_scale)
	{
		case _250dps :
			new_val |= 0x00;
			g_scale_gyro = 131.0;
			break;
		case _500dps :
			new_val |= 0x02;
			g_scale_gyro = 65.5;
			break;
		case _1000dps :
			new_val |= 0x04;
			g_scale_gyro = 32.8;
			break;
		case _2000dps :
			new_val |= 0x06;
			g_scale_gyro = 16.4;
			break;
	}

	write_single_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

static void ICM20948_accel_full_scale_select(accel_scale_t full_scale)
{
	uint8_t new_val = read_single_reg(ub_2, B2_ACCEL_CONFIG);

	switch(full_scale)
	{
		case _2g :
			new_val |= 0x00;
			g_scale_accel = 16384;
			break;
		case _4g :
			new_val |= 0x02;
			g_scale_accel = 8192;
			break;
		case _8g :
			new_val |= 0x04;
			g_scale_accel = 4096;
			break;
		case _16g :
			new_val |= 0x06;
			g_scale_accel = 2048;
			break;
	}

	write_single_reg(ub_2, B2_ACCEL_CONFIG, new_val);
}




void ICM20948_init()
{
	while(!ICM20948_who_am_i());

	ICM20948_device_reset();
	ICM20948_wakeup();

	ICM20948_clock_source(1);
	ICM20948_odr_align_enable();

	ICM20948_spi_slave_enable();

	ICM20948_gyro_low_pass_filter(0);
	ICM20948_accel_low_pass_filter(0);

	ICM20948_gyro_sample_rate_divider(0);
	ICM20948_accel_sample_rate_divider(0);

	//ICM20948_gyro_calibration();
	//ICM20948_accel_calibration();

	ICM20948_gyro_full_scale_select(_2000dps);
	ICM20948_accel_full_scale_select(_16g);
}

void AK09916_init()
{
	ICM20948_i2c_master_reset();
	ICM20948_i2c_master_enable();
	ICM20948_i2c_master_clk_frq(7);

	while(!AK09916_who_am_i());

	AK09916_soft_reset();
	AK09916_operation_mode_setting(continuous_measurement_100hz);
}

bool ICM20948_who_am_i()
{
	uint8_t ICM20948_id = read_single_reg(ub_0, B0_WHO_AM_I);

	if(ICM20948_id == ICM20948_ID)
		return true;
	else
		return false;
}

bool AK09916_who_am_i()
{
	uint8_t AK09916_id = read_single_mag_reg(MAG_WIA2);

	if(AK09916_id == AK09916_ID)
		return true;
	else
		return false;
}

void ICM20948_gyro_read(raw_data_t* data)
{
	uint8_t* temp = read_multiple_reg(ub_0, B0_GYRO_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]);
}

void ICM20948_accel_read(raw_data_t* data)
{
	uint8_t* temp = read_multiple_reg(ub_0, B0_ACCEL_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	// data->z = (int16_t)(temp[4] << 8 | temp[5]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]) + g_scale_accel;
	// Add scale factor because calibraiton function offset gravity acceleration.
}

bool AK09916_mag_read(raw_data_t* data)
{
	uint8_t* temp;
	uint8_t drdy, hofl;

	drdy = read_single_mag_reg(MAG_ST1) & 0x01;
	if(!drdy)	return false;

	temp = read_multiple_mag_reg(MAG_HXL, 6);

	hofl = read_single_mag_reg(MAG_ST2) & 0x08;
	if(hofl)	return false;

	data->x = (int16_t)(temp[1] << 8 | temp[0]);
	data->y = (int16_t)(temp[3] << 8 | temp[2]);
	data->z = (int16_t)(temp[5] << 8 | temp[4]);

	return true;
}

void ICM20948_gyro_read_dps(axises_t* data)
{
	ICM20948_gyro_read(&g_raw_gyro);

	data->x = (float)(g_raw_gyro.x / g_scale_gyro);
	data->y = (float)(g_raw_gyro.y / g_scale_gyro);
	data->z = (float)(g_raw_gyro.z / g_scale_gyro);
}

void ICM20948_accel_read_g(axises_t* data)
{
	ICM20948_accel_read(&g_raw_accel);

	data->x = (float)(g_raw_accel.x / g_scale_accel);
	data->y = (float)(g_raw_accel.y / g_scale_accel);
	data->z = (float)(g_raw_accel.z / g_scale_accel);
}

bool AK09916_mag_read_uT(axises_t* data)
{
	bool new_data = AK09916_mag_read(&g_raw_mag);
	if(!new_data)	return false;

	data->x = (float)(g_raw_mag.x * 0.15);
	data->y = (float)(g_raw_mag.y * 0.15);
	data->z = (float)(g_raw_mag.z * 0.15);
	return true;
}





////////////////////////////
float kp = 200.0f;
float ki = 200.0f;
float kd = 1.5f;
float previous_error = 0.0f;
float integral = 0.0f;
float setpoint = 6.0f;
float dt = 0.01f;
float pitch = 0.0f;
float pitch_a = 0.0f;
float pitch_g = 0.0f;
float roll = 0.0f;
float roll_a = 0.0f;
float roll_g = 0.0f;

float spin = 0;

/////////////////////////////////////////
float y_prev=0;
float Tf_p=0.001f;
float LowPassFilter(float x,float time, float Tf){
	float alpha = Tf/(Tf+time);
	float y =alpha*y_prev + (1.0f - alpha)*x;
	y_prev = y;
	return y;
}

float PID(float pitch) //모터 출력 피아이디
{
	float error = setpoint - pitch;

	integral += error * dt;

	float integral_limit = 1.0f;
	if(integral > integral_limit)
	{
		integral = integral_limit;
	}
	else if(integral < -integral_limit)
	{
		integral = -integral_limit;
	}

	float derivative = (error - previous_error) / dt;

	float output = kp*error + ki*integral + kd*derivative;

	previous_error = error;

	return output;
}
//////////////////////////////////////////////////
//Encoder

float kp_e =2.0f;
float ki_e = 2.0f;
float kd_e = 0.15f;
float previous_error_e = 0.0f;
float integral_e = 0.0f;

float speed_e = 0.0f;

extern int g_Encoder_M1_Now;
extern int g_Encoder_M2_Now;

int encoder[2] = {0};
int speed_e1_c=0;
int speed_e2_c=0;
int speed_e1=0;
int speed_e2=0;
int speed_e1_prev=0;
int speed_e2_prev=0;
int show_encoder = 0;
int setpoint_encoder=0;
/////////////////////////////////////////////////
float PID_e(float error) //조향용 엔코더 피아이디
{


	integral_e += (error-setpoint_encoder) * dt;

	float integral_limit = 4.0f;
	if(integral_e > integral_limit)
	{
		integral_e = integral_limit;
	}
	else if(integral_e < -integral_limit)
	{
		integral_e = -integral_limit;
	}

	float derivative = ((error-setpoint_encoder) - previous_error_e) / dt;

	float output = kp_e*(error-setpoint_encoder) + ki_e*integral_e + kd_e*derivative;

	previous_error_e = error-setpoint_encoder;

	return output;
}

float kp_p =5.0f;
float ki_p = 0.0f;
float kd_p = 2.0f;
float previous_error_p = 0.0f;
float integral_p = 0.0f;
float setvel=0;
float max_setpoint=10.0f;
float min_setpoint=2.0f;

float PID_p(float speed)//for setpoint PID
{
	float error_p = speed+setvel;

	integral_p += error_p * dt;

	float integral_p_limit = 1.0f;
	if(integral_p > integral_p_limit)
	{
		integral_p = integral_p_limit;
	}
	else if(integral_p < -integral_p_limit)
	{
		integral_p = -integral_p_limit;
	}

	float derivative_p = (error_p - previous_error_p) / dt;

	float output_p = (kp_p*error_p + ki_p*integral_p + kd_p*derivative_p)/10000;

	previous_error_p = error_p;

	return output_p;
}
float kp_r =3.0f;
float ki_r = 0.0f;
float kd_r = 0.0f;
float previous_error_r = 0.0f;
float integral_r = 0.0f;
float setpoint_roll=-0.4f;
float rollset=0.6;
float PID_r(float speed) //for roll PID
{
	float error_r= (roll-rollset);

	integral_r += error_r * dt;

	float integral_r_limit = 1.0f;
	if(integral_r > integral_r_limit)
	{
		integral_r = integral_r_limit;
	}
	else if(integral_r < -integral_r_limit)
	{
		integral_r = -integral_r_limit;
	}

	float derivative_r = (error_r - previous_error_r) / dt;

	float output_r = (kp_r*error_r + ki_r*integral_r + kd_r*derivative_r)/10;

	previous_error_r = error_r;

	return output_r;
}

float kp_s =5.0f;
float ki_s = 0.0f;
float kd_s = 0.0f;
float previous_error_s = 0.0f;
float integral_s = 0.0f;

float max_setpoint_s=5.0f;
float min_setpoint_s=-5.0f;

int rservo_p=0;
int rservo_c=0;

////////////////////////////////////////////////////
int rightangle=0;
int leftangle=0;
float servopid=0;
float speed = 0;
float speed_e_pid = 0;
extern int state;

void ICM20948_Read_Data_Handle(void)
{if(state>0){
	print_count++;
	ICM20948_gyro_read_dps(&g_axises_gyro);
	ICM20948_accel_read_g(&g_axises_accel);
	//AK09916_mag_read_uT(&g_axises_mag);

	float ax = g_axises_accel.x;
	float ay = g_axises_accel.y;
	float az = g_axises_accel.z;
	float gx = g_axises_gyro.x;
	float gy = g_axises_gyro.y;
	//	float mx = g_axises_mag.x;
//	float my = g_axises_mag.y;
//	float mz = g_axises_mag.z;

	roll_g = roll + gy*0.01;
	roll_a = atan(-ax/sqrt(pow(ax, 2)+pow(az, 2))) * 180.0/M_PI;
	roll = 0.98*roll_g + 0.02*roll_a;

	pitch_g = pitch + gx*0.01;
	pitch_a = atan(ay/az) * 180.0/M_PI;
	pitch = 0.98*pitch_g + 0.02*pitch_a;
//	//서보 roll pid
//			servopid=PID_r(roll);
//			if(servopid>40){servopid=40;}
//			else if(servopid<-40){servopid=-40;}
//			if(servopid>=0){
//				if(leftangle==0){
//					rightangle=servopid;
//					if(rightangle>40){rightangle=40;}
//					else if(rightangle<0){rightangle=0;}
//				}
//				else{
//					leftangle=-servopid;
//					if(leftangle>40){leftangle=40;}
//					else if(leftangle<0){leftangle=0;}
//				}
//			}
//			else if(servopid<0){
//				if(rightangle==0){
//					leftangle=-servopid;
//					if(leftangle>40){leftangle=40;}
//					else if(leftangle<0){leftangle=0;}
//				}
//				else{
//					rightangle=-servopid;
//					if(rightangle>40){rightangle=40;}
//					else if(rightangle<0){rightangle=0;}
//				}
//			}

	speed = PID(pitch);
//	float speed1 = PID(pitch);
//	speed=LowPassFilter(speed1,dt,Tf_p);

	Encoder_Get_ALL(encoder);
	speed_e = encoder[1] - encoder[0];
	speed_e_pid = PID_e(speed_e);
//	show_encoder++;
//	if (show_encoder > 10)
//	{
//		show_encoder = 0;
//		printf("Encoder:%d, %d, %f, %f \r\n\n\n\n\n\n", encoder[0], encoder[1], speed_e, speed_e_pid);
//	}
	if(g_Encoder_M1_Now > 50000 && g_Encoder_M2_Now > 50000)
	{
		g_Encoder_M1_Now -= 50000;
		g_Encoder_M2_Now -= 50000;
	}
	Encoder_Update_Count();

	if(print_count==5){
		//밸런싱 셋포인트 pid
		speed_e1=encoder[0];
		speed_e2=encoder[1];

		speed_e1_c=speed_e1-speed_e1_prev;
		speed_e2_c=speed_e2-speed_e2_prev;

		speed_e1_prev=speed_e1;
		speed_e2_prev=speed_e2;
		setpoint+=PID_p(speed_e1_c+speed_e2_c);

		//서보 roll pid
				servopid=PID_r(roll);
				if(servopid>40){servopid=40;}
				else if(servopid<-40){servopid=-40;}
				if(servopid>=1){
					if(leftangle==0){
						rightangle+=servopid;
						if(rightangle>40){rightangle=40;}
						else if(rightangle<0){rightangle=0;}
					}
					else{
						leftangle-=servopid;
						if(leftangle>40){leftangle=40;}
						else if(leftangle<0){leftangle=0;}
					}
				}
				else if(servopid<=-1){
					if(rightangle==0){
						leftangle-=servopid;
						if(leftangle>40){leftangle=40;}
						else if(leftangle<0){leftangle=0;}
					}
					else{
						rightangle+=servopid;
						if(rightangle>40){rightangle=40;}
						else if(rightangle<0){rightangle=0;}
					}
				}
		if(setpoint>max_setpoint){setpoint=max_setpoint;}
		else if(setpoint<min_setpoint){setpoint=min_setpoint;}
		//PwmServo_Set_Angle_All(40-leftangle,140+rightangle, 30, 30);
		print_count=0;
	}
	if(pitch <35 && pitch > -30)
	{
		if(spin == 0){
			Motor_Set_Pwm(MOTOR_ID_M1, speed+speed_e_pid);
			Motor_Set_Pwm(MOTOR_ID_M2, -(speed-speed_e_pid));
			PwmServo_Set_Angle_All(40-rightangle,140+leftangle, 30, 30);
		}
		else{
			Motor_Set_Pwm(MOTOR_ID_M1, speed);
			Motor_Set_Pwm(MOTOR_ID_M2, -(speed));
		}

	}
	else {Motor_Stop(0);}
}}







