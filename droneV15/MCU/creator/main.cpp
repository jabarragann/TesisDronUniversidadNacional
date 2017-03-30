/*
 * Copyright 2016 <Admobilize>
 * MATRIX Labs  [http://creator.matrix.one]
 * This file is part of MATRIX Creator firmware for MCU
 * Author: Andrés Calderón [andres.calderon@admobilize.com]
 *
 * MATRIX Creator firmware for MCU is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ch.h"
#include "hal.h"
#include "board.h"

#include <math.h>
#include <string.h>
#include <mcuconf.h>

#include "./i2c.h"
#include "./sensors_data.h"
#include "./mpl3115a2.h"
#include "./lsm9ds1.h"
#include "./hts221.h"
#include "./veml6070.h"

#include "chprintf.h"

extern "C" {
#include "atmel_psram.h"
}

const uint32_t kFirmwareCreatorID = 0x10;
const uint32_t kFirmwareVersion = 0xB9B9B9B9;  //0xYYMMDD 


//Global Variables
int32_t maxPwm=0x04BAF0;
int32_t minPwm=0x035b60;
IMUData dataimu;

char getChar(uint32_t data, char select)
{
	char dataToReturn=0;
	
	switch (select)
	{
	case 1:
	 dataToReturn=(data & 0xFF0000)>>16;
	 
	break;
	case 2:
	 dataToReturn=(data & 0x00FF00)>>8;
	break;
	case 3:
	 dataToReturn=(data & 0x0000FF);
	break;
	default:
	break;
   }
return dataToReturn; 
}
	
/////////////////////////////////////////
////LecturaMemoriaDoblePuerto////////////
/////////////////////////////////////////
char *ptr = (char *) PSRAM_BASE_ADDRESS;

/* Global objects */
creator::I2C i2c;  

void psram_copy(uint32_t mem_offset, char *data, uint8_t len) {
  register char *psram = (char *)PSRAM_BASE_ADDRESS;

  for (int i = 0; i < len; i++) {
    psram[mem_offset + i] = data[i];
  }
}
//////////////////////////////////////

////////Funciones PWM Gabriela////////
void set_period (PWMData *data,char d1, char d2, char d3){ 
   // Periodo de 20ms- f --> 50Hz para clk 200MHz 
   data->period_1=d1;	
   data->period_2=d2;
   data->period_3=d3;
   
   psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
}
void set_duty (PWMData *data,char motor,char d1, char d2, char d3){
	switch(motor){
	case 1:
	data->duty1_1   = d1;
   	data->duty1_2   = d2;
   	data->duty1_3   = d3;
   	psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
   	break;
   	case 2:
	data->duty2_1   = d1;
    data->duty2_2   = d2;
	data->duty2_3   = d3;
	psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
	break;
	case 3:
	data->duty3_1   = d1;
   	data->duty3_2   = d2;
   	data->duty3_3   = d3;
   	psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
   	break;
   	case 4:
   	data->duty4_1   = d1;
        data->duty4_2   = d2;
        data->duty4_3   = d3;
        psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
   	break;
   	default:
   	data->duty1_1   = 0;
   	data->duty1_2   = 0;
   	data->duty1_3   = 0;
   	psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
	}
  }


void initMotors(PWMData *data)
{
	//Set period to 20ms and duty cycle of all motors to 1ms
    set_period(data,0x3D,0X09,0X00);
    set_duty (data,1,0x03,0X0D,0X40);
    set_duty (data,2,0x03,0X0D,0X40);
    set_duty (data,3,0x03,0X0D,0X40);
    set_duty (data,4,0x03,0X0D,0X40);
    chThdSleepMilliseconds(3000);    	
}
//Aumento secuencial de ciclo útil en 20 pasos 1m-2m
void plusDuty(PWMData *data,int motor){
//set_period(data,0x3D,0X09,0X00);
int a =ptr[0xF1];
switch(a){
	case 0:  set_duty (data,motor,0x03,0X0D,0X40); break;
	case 1:  set_duty (data,motor,0x03,0X34,0X50); break;
	case 2:  set_duty (data,motor,0x03,0X5B,0X60); break;
	case 3:  set_duty (data,motor,0x03,0X82,0X70); break;
	case 4:  set_duty (data,motor,0x03,0XA9,0X80); break;
	case 5:  set_duty (data,motor,0x03,0XD0,0X90); break;
	case 6:  set_duty (data,motor,0x03,0XF7,0XA0); break;
	case 7:  set_duty (data,motor,0x04,0x1E,0xB0); break;
	case 8:  set_duty (data,motor,0x04,0X45,0XC0); break;
	case 9:  set_duty (data,motor,0x04,0X6C,0XD0); break;
	case 10: set_duty (data,motor,0x04,0X93,0XE0); break;
	case 11: set_duty (data,motor,0x04,0XBA,0XF0); break;
	case 12: set_duty (data,motor,0x04,0XE2,0X00); break;
	case 13: set_duty (data,motor,0x05,0X09,0X10); break;
	case 14: set_duty (data,motor,0x05,0X30,0X20); break;
	case 15: set_duty (data,motor,0x05,0X57,0X30); break;
	case 16: set_duty (data,motor,0x05,0X7E,0X40); break;
	case 17: set_duty (data,motor,0x05,0XA5,0X50); break;
	case 18: set_duty (data,motor,0x05,0XCC,0X60); break;
	case 19: set_duty (data,motor,0x05,0XF3,0X70); break;
	case 20: set_duty (data,motor,0x06,0X1A,0X80); break;
	default: set_duty (data,motor,0x03,0X0D,0X40);
	}
}
int32_t plusDuty2(){

int a =ptr[0xF1];
switch(a){
	case 0:  return 0x030D40; break;
	case 1:  return 0x033450; break;
	case 2:  return 0x035B60; break;
	case 3:  return 0x038270; break;
	case 4:  return 0x03A980; break;
	case 5:  return 0x03D090; break;
	case 6:  return 0x03F7A0; break;
	case 7:  return 0x041EB0; break;
	case 8:  return 0x0445C0; break;
	case 9:  return 0x046CD0; break;
	case 10: return 0x0493E0; break;
	case 11: return 0x04BAF0; break;
	case 12: return 0x04E200; break;
	case 13: return 0x050910; break;
	case 14: return 0x053020; break;
	case 15: return 0x055730; break;
	case 16: return 0x057E40; break;
	case 17: return 0x05A550; break;
	case 18: return 0x05CC60; break;
	case 19: return 0x05F370; break;
	case 20: return 0x061A80; break;
	default: return 0x030D40; break;
	}
}

static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {
  (void)arg;
  
  sdStart(&SD1,NULL);
  
  char x;
  for (x=0;x<32;x++)
    {
		ptr[0xE0+x]=0xBB;
	}
  while (TRUE) {
    palClearPad(IOPORT3, 17);
    chThdSleepMilliseconds(1000);
    
    
    char y;
    for (x=0;x<9;x++)
    {
		y=ptr[0x200+x];
		ptr[0xF0+x]=y;
	}
    
    ptr[0xEF]=0xCC;
    
    char i,j;
    for (j=0; j< 16; j++){
      for (i = 0; i < 16; ++i) {
        chprintf((BaseChannel *)&SD1, "%x:%x \t",(j*16+i),(ptr[j*16 + i] & 0xFF)) ;
      }
      chprintf((BaseChannel *)&SD1, "\n\r" ) ;
    }
    chprintf((BaseChannel *)&SD1, "\n\r" ) ;
    
    palSetPad(IOPORT3, 17);
    chThdSleepMilliseconds(1000);
  }
  return(0);
}

static WORKING_AREA(waIMUThread, 512);
static msg_t IMUThread(void *arg) {
  (void)arg;
  
	//LSM9DS1 imu(&i2c, IMU_MODE_I2C, 0x6A, 0x1C);
	//imu.begin();
	//IMUData dataimu;
	PWMData data;
	initMotors(&data);  
	return (0);
}
/*
 * Application entry point.
 */
int main(void) 
{
	halInit();

	chSysInit();

	/* Configure EBI I/O for psram connection*/
	PIO_Configure(pinPsram, PIO_LISTSIZE(pinPsram));

	/* complete SMC configuration between PSRAM and SMC waveforms.*/
	BOARD_ConfigurePSRAM(SMC);

	i2c.Init();


	/* Creates the imu thread. */
	//chThdCreateStatic(waIMUThread, sizeof(waIMUThread), NORMALPRIO, IMUThread,
	//                 NULL);
									  
	//chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

	//Initialize   
	chThdSleepMilliseconds(20);                           
	palSetPad(IOPORT3, 17);
	sdStart(&SD1,NULL);
	
	//Initialize IMU
	LSM9DS1 imu(&i2c, IMU_MODE_I2C, 0x6A, 0x1C);	
	imu.begin();
	IMUData dataimu;

	
    //Constants
	float refreshRate=0.01;            				 //Time between different measures of the gyroscope. F=100Hz
	float pitchGy=0,rollGy=0,yawGy=0;		         //angles calculated from the gyroscope.
	float pitchRate=0,rollRate=0,yawRate=0;
	
	//Gyro average values of 2000 measurements  
	float gyro_x_ave=0,gyro_y_ave=0,gyro_z_ave=0;
	for (int cal_int =0;cal_int<2000;cal_int ++)
	{
		imu.readGyro();
		gyro_x_ave+=imu.calcGyro(imu.gx);
		gyro_y_ave+=imu.calcGyro(imu.gy);
		gyro_z_ave+=imu.calcGyro(imu.gz);
	}
	gyro_x_ave/=2000;
	gyro_y_ave/=2000;
	gyro_z_ave/=2000;

	while (true) 
	{
		//imu calculations

		//Yawn  --> psi 
		//Pitch --> theta
		//Roll  --> phi
		//p     --> gyro_x
		//q     --> gyro_y
		//r     --> gyro_z

		//Gyroscope Calculations
		
		//Substract from the gyro measurement the corresponding average value.
		imu.readGyro();
		dataimu.gyro_x = imu.calcGyro(imu.gx)-gyro_x_ave;
		dataimu.gyro_y = imu.calcGyro(imu.gy)-gyro_y_ave;
		dataimu.gyro_z = imu.calcGyro(imu.gz)-gyro_z_ave;

		//Rotational Matrices to obtain inertial frame data. RollGy angles can never be 90 degrees.
		rollRate  = dataimu.gyro_x + dataimu.gyro_y*sin(rollGy)*tan(pitchGy)+dataimu.gyro_z*cos(rollGy)*tan(pitchGy); 
		pitchRate = dataimu.gyro_y*cos(rollGy)-dataimu.gyro_z*sin(rollGy);
		yawRate   = dataimu.gyro_y*sin(rollGy)/cos(pitchGy) + dataimu.gyro_z*cos(rollGy)/cos(pitchGy);
		
		//Calculate pitch,roll and yawn from 
		pitchGy+=pitchRate*refreshRate;
		rollGy +=rollRate*refreshRate;
		yawGy  +=yawRate*refreshRate;


		//Magnetometer
		imu.readMag();
		dataimu.mag_x = imu.calcMag(imu.mx);
		dataimu.mag_y = imu.calcMag(imu.my);
		dataimu.mag_z = imu.calcMag(imu.mz);

		//Accelerometer
		imu.readAccel();
		dataimu.accel_x = imu.calcAccel(imu.ax);
		dataimu.accel_y = imu.calcAccel(imu.ay);
		dataimu.accel_z = imu.calcAccel(imu.az);

		//Loading data
		dataimu.yaw =yawGy;
		dataimu.roll=rollGy;
		dataimu.pitch=pitchGy;	
	}
	
	return (0);
}
