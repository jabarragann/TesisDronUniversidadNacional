#include <math.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <iomanip>

#include "../cpp/driver/wishbone_bus.h"
#include "../cpp/driver/fw_data.h"
#include "../cpp/driver/mcu_firmware.h"
#include "../cpp/driver/imu_data.h"
#include "../cpp/driver/imu_sensor.h"
#include "../cpp/driver/pressure_data.h"
#include "../cpp/driver/pressure_sensor.h"
#include "../cpp/driver/humidity_data.h"
#include "../cpp/driver/humidity_sensor.h"
#include "../cpp/driver/uv_sensor.h"
#include "../cpp/driver/uv_data.h"

namespace hal = matrix_hal;

int main() {
  hal::WishboneBus bus;
  bus.SpiInit();

  hal::IMUSensor imu_sensor;
  imu_sensor.Setup(&bus);

  hal::IMUData imu_data;
  
  std::cout<<"Calibrating Gyro"<<std::endl;
  
  float const1=3.14/(250*180);
  float const2=180/3.14;
  
  float gyro_x_cal,gyro_y_cal,gyro_z_cal;
  
  float pitchGy,yawGy,rollGy;
  float pitchAcc,yawAcc,rollAcc;
  
  float gyroX,gyroY,gyroZ;
  float accX,accY,accZ;
  
  pitchGy=0;
  rollGy=0;
  yawGy=0;
  gyro_x_cal=0;
  gyro_y_cal=0;
  gyro_z_cal=0;
  
  
  for (int cal_int =0;cal_int<2000;cal_int ++)
  {
	  if(cal_int % 125 ==0) printf("* ");
	  imu_sensor.Read(&imu_data);
	  gyro_x_cal+=imu_data.gyro_x;
	  gyro_y_cal+=imu_data.gyro_y;
	  gyro_z_cal+=imu_data.gyro_z;
	  usleep(3000);
  }
  gyro_x_cal/=2000;
  gyro_y_cal/=2000;
  gyro_z_cal/=2000;
	  
  std::cout<<std::endl;
  
  int counter=0;
  
  
  while (true) {
	  
    imu_sensor.Read(&imu_data); 
    
    //Gyroscope calculations
    
    gyroX=imu_data.gyro_x-gyro_x_cal;
    gyroY=imu_data.gyro_y-gyro_y_cal;
    gyroZ=imu_data.gyro_z-gyro_z_cal;
    
    pitchGy+=gyroX*0.004;
    rollGy +=gyroY*0.004;
    yawGy  +=gyroZ*0.004;
    
    rollGy -=pitchGy*sin(gyroZ*const1);
    pitchGy+=rollGy *sin(gyroZ*const1);
    
    //Accelerometer Calculations
    accX=imu_data.accel_x;
    accY=imu_data.accel_y;
    accZ=imu_data.accel_z;
    
    float grav_vector=sqrt(accX*accX+accY*accY+accZ*accZ);
    pitchAcc=asin(accY/grav_vector)*const2;
    rollAcc=-asin(accX/grav_vector)*const2;
    yawAcc=0;
    
    //totalCalculations
    
    //printing
    counter++;
    if(counter%50==0)
    {    
		std::system("clear");
		printf("\nAngular Velocity\n");
		printf("Vx: %7.3f, Vy: %7.3f, Vz: %7.3f deg/s\n",gyroX,gyroY,gyroZ);
		printf("Pitch: %7.3f, Roll: %7.3f, Yaw: %7.3f deg\n",pitchGy,rollGy,yawGy);
		
		printf("\nAcceleration\n");
		printf("Vx: %7.3f, Vy: %7.3f, Vz: %7.3f g\n",accX,accY,accZ);
		printf("Pitch: %7.4f, Roll: %7.4f, Yaw: %7.4f deg\n",pitchAcc,rollAcc,yawAcc);
		
		counter%=50;
	}	

/*	
	std::cout <<"Acceleration"<<std::endl;
	std::cout<<"Ax: "<< imu_data.accel_x<<"Ay: "<<imu_data.accel_y <<"Az "<< imu_data.accel_z<< "g "<<std::endl;
	
	std::cout <<"Magnetic intensity"<<std::endl;
	std::cout<<"Mx: "<< imu_data.mag_x<<"My: "<<imu_data.mag_y <<"Mz "<< imu_data.mag_z<< "Gausss"<<std::endl<<std::endl;
*/
    usleep(4000);
  }

  return 0;
}
