#include "AP_OpticalFlow_VK.h"

extern const AP_HAL::HAL& hal;

#define rpi_I2C_ADDRESS		0x13

AP_OpticalFlow_VK::AP_OpticalFlow_VK(){}

void AP_OpticalFlow_VK::init()
{
    dev = std::move(hal.i2c_mgr->get_device(1, rpi_I2C_ADDRESS));
    if (!dev) {
        return;
    }

    sem = hal.util->new_semaphore();

    dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_OpticalFlow_VK::read_sens, void));
};

void AP_OpticalFlow_VK::read_sens(void)
{

	uint8_t rpicall[1] = {0x74};

	if (!dev->transfer(rpicall, 1 ,nullptr , 0)){
		return;
	}

    if (dev->transfer(nullptr, 0, (uint8_t*)&data_buf, sizeof(data_buf))){
    	_last_read_ms = AP_HAL::millis();
	}
	else{
		return;
	}

    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        timestamp = AP_HAL::millis();
        sem->give();
    }


};

// retrieve latest sensor data - returns true if new data is available
bool AP_OpticalFlow_VK::update()
{
    bool new_data = false;
    if (!dev || !sem) {
        return false;
    }
    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (_last_update_ms != timestamp) {
            new_data = true;
        }
//    	hal.console->printf(("n1: %d\r\n"),new_data);
        uint32_t sum = 0;
        for(int i = 0 ; i < 12 ; i++){
            sum += data_buf[i];
        }
        uint32_t check_sum = *(uint32_t*)(&data_buf[12]);
        if(check_sum != sum || check_sum == 0){
            new_data = false;
        }
//    	hal.console->printf(("n2: %d\r\n"),new_data);
        _last_update_ms = timestamp;
        opt_healthy = (AP_HAL::millis() - _last_read_ms < 500);
        sem->give();
//    	hal.console->printf(("n3: %d\r\n"),opt_healthy);
//        for(int i = 0 ; i < 12 ; i+=4){
//        	hal.console->printf(("%f,"),(*(float*)(&data_buf[i])));
//        }
//        hal.console->printf(("n3:%d\r\n"),opt_healthy);
    }
    // return true if new data found
    return new_data;
};

void AP_OpticalFlow_VK::get_data(float& x,float& y,float& z)
{
    x = *(float*)(&data_buf[0]);
    y = *(float*)(&data_buf[4]);
    z = *(float*)(&data_buf[8]);
};

void AP_OpticalFlow_VK::get_data(Vector3f &v){
	this->get_data(v.x,v.y,v.z);
}
