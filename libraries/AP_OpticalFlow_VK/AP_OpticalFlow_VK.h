#pragma once
#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <utility>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/vector3.h>

class AP_OpticalFlow_VK
{
public:

	AP_OpticalFlow_VK();

    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    AP_HAL::Semaphore *sem;

    void init();

    void read_sens(void);

    // true if opt sensor is online and healthy
    bool healthy() const { return opt_healthy; }

    // timestamp of most recent data read from the sensor
    uint32_t last_update_ms() const { return _last_update_ms; }

    // retrieve latest sensor data - returns true if new data is available
    bool update();
    
    bool get_data(float& x,float& y,float& z);

    bool get_data(Vector3f &v);

    void read_optical_kv(void);

    bool opt_healthy = 1; // true if sensor is healthy
    bool is_err =0;

    uint8_t data_buf[16];

    uint32_t _last_update_ms;
    uint32_t _last_read_ms;
	uint32_t timestamp;

};
