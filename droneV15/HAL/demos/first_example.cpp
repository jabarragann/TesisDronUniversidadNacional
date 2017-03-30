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

#include "../cpp/driver/imu_sensor.h"
#include "../cpp/driver/creator_memory_map.h"


namespace hal = matrix_hal;

const uint16_t kMCUBaseAddress = 0x3800;
const uint16_t kMemoryOffsetMCU = 0x90;
const uint16_t kMemoryOffsetSamCommunication=0xD0;

struct TestData
{
	uint32_t period;
	uint32_t dutyCycle1;
	uint32_t dutyCycle2;
	uint32_t dutyCycle3;
};

int main() 
{
	hal::WishboneBus bus;
	bus.SpiInit();
	
	TestData test;
	
    unsigned char wb_data_buffer[2];
    wb_data_buffer[0]=0x7B;
    wb_data_buffer[1]=0x7A;

	uint32_t i;
	while(true)
	{
		for (i=0;i<16;i++)
		{
			bus.SpiRead(kMCUBaseAddress + (kMemoryOffsetMCU >> 1),(unsigned char*) &test, sizeof(TestData));
			
			std::cout << "Hello World, " << i<<"\n"<<std::endl;
			std::cout << "Period = 0x" <<  std::hex <<  test.period  << std::endl;
			std::cout << "DutyCycle1 = 0x" <<  std::hex <<  test.dutyCycle1 << std::endl;
			std::cout << "DutyCycle2 = 0x" <<  std::hex <<  test.dutyCycle2  << std::endl;
			std::cout << "DutyCycle3 = 0x" <<  std::hex <<  test.dutyCycle3  << std::endl;
			
			bus.SpiWrite(kMCUBaseAddress+(kMemoryOffsetSamCommunication>>1), wb_data_buffer,0);
			usleep(1000000);
		}
	}
}
