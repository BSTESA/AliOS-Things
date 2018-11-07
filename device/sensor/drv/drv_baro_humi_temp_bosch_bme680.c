/*********************************************************************************************
*
*Copyright (C) 2016 - 2022 Bosch Sensortec GmbH

*Redistribution and use in source and binary forms, with or without
*modification, are permitted provided that the following conditions are met:

*Redistributions of source code must retain the above copyright
*notice, this list of conditions and the following disclaimer.

*Redistributions in binary form must reproduce the above copyright
*notice, this list of conditions and the following disclaimer in the
*documentation and/or other materials provided with the distribution.

*Neither the name of the copyright holder nor the names of the
*contributors may be used to endorse or promote products derived from
*this software without specific prior written permission.

*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
*CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
*IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
*OR CONTRIBUTORS BE LIABLE FOR ANY
*DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
*OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
*PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
*WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*ANY WAY OUT OF THE USE OF THIS
*SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

*The information provided is believed to be accurate and reliable.
*The copyright holder assumes no responsibility
*for the consequences of use
*of such information nor for any infringement of patents or
*other rights of third parties which may result from its use.
*No license is granted by implication or otherwise under any patent or
*patent rights of the copyright holder.
*
*
*******************************************************************************************/
/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */
#include <aos/aos.h>
#include <hal/base.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vfs_conf.h>
#include <vfs_err.h>
#include <vfs_register.h>
#include "bme680.h"
#include "common.h"
#include "hal/sensor.h"


#define BME680_CHIP_ID_VAL		   (0x61)
#define BME680_CHIP_ID_ADDR        (0xD0)
#define BME680_I2C_SLAVE_ADDR_LOW  (0X76)
#define BME680_I2C_SLAVE_ADDR_HIGH (0X77)
#define BME680_I2C_ADDR_TRANS(n)   ((n)<<1)  
#define BME680_I2C_ADDR            BME680_I2C_ADDR_TRANS(BME680_I2C_SLAVE_ADDR_LOW)

struct bme680_dev g_bme680sensor;
static int32_t    g_bme680flag = 0;

i2c_dev_t bme680_ctx = {
    .port = 3,
    .config.address_width = 8,
    .config.freq = 400000,
    .config.dev_addr = BME680_I2C_ADDR,
};

/**
 * This function validates the chip ID of device
 * 
 * @param[in]  drv  pointer to the i2c dev
 * @param[in]  id_value  the expected CHIPID
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_baro_bosch_bme680_validate_id(i2c_dev_t* drv, uint8_t id_value)
{
    int     ret = 0;
    uint8_t value = 0;

    if (drv == NULL) {
        return -1;
    }
    
    ret = sensor_i2c_read(drv,
                          BME680_CHIP_ID_ADDR,
                          &value, 
                          I2C_DATA_LEN, 
                          I2C_OP_RETRIES);
    if (unlikely(ret) != 0) {
        return ret;
    }
    
    if (id_value != value) {
        return -1;
    }
    return 0;
}

/**
 * This function reads the register in device
 * 
 * @param[in]  dev_id    the i2c dev slave address
 * @param[in]  reg_addr  the address of the register
 * @param[in]  data      ptr to data that is read
 * @param[in]  len       length of data
 * @return  the operation status, 0 is OK, others is error
 */
int8_t bme680_I2C_read(
       uint8_t dev_id,
       uint8_t reg_addr,
       uint8_t *data,
       uint16_t len)
{
	int ret = 0;
	if (&bme680_ctx == NULL){
        return -1;
    }

    ret = sensor_i2c_read(&bme680_ctx,reg_addr, data, len, I2C_OP_RETRIES);
    if (unlikely(ret) != 0) {
        return ret;
    }

	return 0;
}

/* bme680 I2C write user function*/
/**
 * This function writes the register in device
 * 
 * @param[in]  dev_id    the i2c dev slave address
 * @param[in]  reg_addr  the address of the register
 * @param[in]  data      ptr to data that is written
 * @param[in]  len       length of data
 * @return  the operation status, 0 is OK, others is error
 */
int8_t bme680_I2C_write(
       uint8_t dev_id, 
       uint8_t reg_addr,
       uint8_t *data,
       uint16_t len)
{
	int ret = 0;
	if (&bme680_ctx == NULL){
        return -1;
    }

	ret = sensor_i2c_write(&bme680_ctx, reg_addr, data, len, I2C_OP_RETRIES);
    if (unlikely(ret) != 0) {
        return ret;
    }

	return 0;
}

/* bme680 delay user function in ms*/
void bme680_delay(uint32_t period)
{
    aos_msleep(period);
}

/**
 * This function sets the baro powermode
 * 
 * @param[in]  drv   pointer to the i2c dev
 * @param[in]  mode  the powermode to be setted
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_baro_humi_temp_bosch_bme680_set_power_mode(
           i2c_dev_t* drv,
           dev_power_mode_e mode)
{
    int     ret = 0;
	int8_t  rslt = BME680_OK;
    uint8_t value = 0x00;
    uint8_t dev_mode;
    
    switch(mode){
        case DEV_POWER_OFF:
        case DEV_SLEEP:{
            dev_mode = (uint8_t)BME680_SLEEP_MODE;

            break;
            }
        case DEV_POWER_ON:{
            dev_mode = (uint8_t)BME680_FORCED_MODE;
            break;
            }
        default:return -1;
    }

	/* Select the power mode */
	g_bme680sensor.power_mode = dev_mode;

	/* Set the power mode */
	rslt = bme680_set_sensor_mode(&g_bme680sensor);
	if (rslt != BME680_OK) {
	    LOG("bme680_set_sensor_settings failed, rslt = %d \n", rslt);
		ret = -2;
	}

    if (unlikely(ret) != 0) {
        return ret;
    }
    
    return 0;
}

/**
 * This function sets the sensor to default
 * 
 * @param[in]  drv  pointer to the i2c dev
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_baro_humi_temp_bosch_bme680_set_default_config(i2c_dev_t* drv)
{
    int      ret = 0;
	int8_t   rslt = BME680_OK;
	uint16_t meas_period;
	uint8_t  set_required_settings;
	/* Set the temperature, pressure and humidity settings */
	g_bme680sensor.tph_sett.os_hum = BME680_OS_2X;
	g_bme680sensor.tph_sett.os_pres = BME680_OS_4X;
	g_bme680sensor.tph_sett.os_temp = BME680_OS_8X;
	g_bme680sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

	/* Set the remaining gas sensor settings and link the heating profile */
	g_bme680sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
	/* Create a ramp heat waveform in 3 steps */
	g_bme680sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
	g_bme680sensor.gas_sett.heatr_dur = 150; /* milliseconds user can change the heat time here*/

	/* Select the power mode */
	/* Must be set before writing the sensor configuration */
	g_bme680sensor.power_mode = BME680_FORCED_MODE;

	/* Set the required sensor settings needed */
	set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL 
		| BME680_GAS_SENSOR_SEL;

	/* Set the desired sensor configuration */
	rslt = bme680_set_sensor_settings(set_required_settings,&g_bme680sensor);
	if (rslt != BME680_OK) {
	    LOG("bme680_set_sensor_settings failed, rslt = %d \n", rslt);
		ret = -1;
	}

	/* Set the power mode */
	rslt = bme680_set_sensor_mode(&g_bme680sensor);
	if (rslt != BME680_OK) {
	    LOG("bme680_set_sensor_settings failed, rslt = %d \n", rslt);
		ret = -2;
	}

	/* Get the total measurement duration so as to sleep or wait till the
	 * measurement is complete */
	bme680_get_profile_dur(&meas_period, &g_bme680sensor);
	aos_msleep(meas_period); /* Delay till the measurement is ready */

     if (unlikely(ret) != 0) {
        return ret;
    }

    return 0;
}

/**
 * This function opens the baro 
 * 
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_baro_bosch_bme680_open(void)
{
    int ret = 0;

   /* open pressure temperature humidity */
    ret = drv_baro_humi_temp_bosch_bme680_set_default_config(&bme680_ctx);
    if (unlikely(ret) != 0) {
        return ret;
    }
    ret  =  drv_baro_humi_temp_bosch_bme680_set_power_mode(&bme680_ctx, DEV_POWER_ON);
    if (unlikely(ret) != 0) {
        return -1;
    }
    
    LOG("%s %s successfully \n", SENSOR_STR, __func__);
    return 0;

}

/**
 * This function closes the baro 
 * 
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_baro_bosch_bme680_close(void)
{
    int ret = 0;
    ret  = drv_baro_humi_temp_bosch_bme680_set_power_mode(
           &bme680_ctx,
           DEV_POWER_OFF);
    if (unlikely(ret) != 0) {
        return -1;
    }
    LOG("%s %s successfully \n", SENSOR_STR, __func__);
    return 0;
}

/**
 * This function reads the baro data and reports the data 
 * 
 * @param[in out]  buf   buffer for baro data
 * @param[in out]  len   length of data
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_baro_bosch_bme680_read(void *buf, size_t len)
{
    int                      ret = 0;
    size_t                   size = 0;
    barometer_data_t*        pdata = (barometer_data_t*)buf;
    struct bme680_field_data data;
    int8_t                   rslt = BME680_OK;
    if (buf == NULL) {
        return -1;
    }
    size = sizeof(barometer_data_t);
    if (len < size) {
        return -1;
    }
    
	ret = drv_baro_humi_temp_bosch_bme680_set_default_config(&bme680_ctx);
    if (unlikely(ret) != 0) {
        return -1;
    }

    rslt = bme680_get_sensor_data(&data, &g_bme680sensor);
	if (rslt != BME680_OK)
	    LOG("bme680_get_sensor_data failed,  rslt = %d \n", rslt);
  
  	printf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
	data.pressure / 100.0f, data.humidity / 1000.0f );
	/* Avoid using measurements from an unstable heating setup */
	if(data.status & BME680_GASM_VALID_MSK)
		printf(", G: %d ohms", data.gas_resistance);

	pdata->p = data.pressure;
    if (unlikely(ret) != 0) {
        return -1;
    }

    pdata->timestamp = aos_now_ms();
    
    return (int)size;
}

/**
 * This function writess the baro
 * 
 * @param[in out]  buf   buffer for written data
 * @param[in out]  len   length of data
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_baro_bosch_bme680_write(const void *buf, size_t len)
{
    (void)buf;
    (void)len;
    return 0;
}

/**
 * This function is for the baro ioctl
 * 
 * @param[in]  cmd   the ioctl command
 * @param[in]  arg   the correspondding parameter
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_baro_bosch_bme680_ioctl(int cmd, unsigned long arg)
{
    int ret = 0;
    
    switch(cmd){
        case SENSOR_IOCTL_ODR_SET:{
        }break;
        case SENSOR_IOCTL_SET_POWER:{
            ret = drv_baro_humi_temp_bosch_bme680_set_power_mode(
                  &bme680_ctx,
                  arg);
            if (unlikely(ret) != 0) {
                return -1;
            }
        }break;
        case SENSOR_IOCTL_GET_INFO:{ 
            /* fill the dev info here */
            dev_sensor_info_t *info = (dev_sensor_info_t *)arg;
            info->model = "BME680";
            info->range_max = 1100;
            info->range_min = 300;
            info->unit = pa;

        }break;
       
       default:break;
    }

    LOG("%s %s successfully \n", SENSOR_STR, __func__);
    return 0;
}

/**
 * This function is the ISR 
 * 
 * @return
 */
static void drv_baro_bosch_bme680_irq_handle(void)
{
    /* no handle so far */
}

/**
 * This function is for the baro initialization
 * 
 * @return  the operation status, 0 is OK, others is error
 */
int drv_baro_bosch_bme680_init(void)
{
    int          ret = 0;
    sensor_obj_t sensor;
	int8_t       rslt = BME680_OK;
    
	g_bme680sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
	g_bme680sensor.intf = BME680_I2C_INTF;
    g_bme680sensor.read = bme680_I2C_read;
    g_bme680sensor.write = bme680_I2C_write;
    g_bme680sensor.delay_ms = bme680_delay;
    g_bme680sensor.amb_temp = 25;

    /* fill the sensor obj parameters here */
    sensor.tag = TAG_DEV_BARO;
    sensor.path = dev_baro_path;
    sensor.io_port = I2C_PORT;
    sensor.open = drv_baro_bosch_bme680_open;
    sensor.close = drv_baro_bosch_bme680_close;
    sensor.read = drv_baro_bosch_bme680_read;
    sensor.write = drv_baro_bosch_bme680_write;
    sensor.ioctl = drv_baro_bosch_bme680_ioctl;
    sensor.irq_handle = drv_baro_bosch_bme680_irq_handle;

    ret = sensor_create_obj(&sensor);
    if (unlikely(ret) != 0) {
        return -1;
    }

    if (g_bme680flag == 0) {
        rslt = bme680_init(&g_bme680sensor);
	    if(rslt != BME680_OK)
	    {
	        LOG("bme680_init failed,  rslt = %d \n", rslt);
		    return -1;
	    }
        ret = drv_baro_humi_temp_bosch_bme680_set_default_config(&bme680_ctx);
        if (unlikely(ret) != 0) {
            return -1;
        }
        g_bme680flag = 1;
   }

    LOG("%s %s successfully \n", SENSOR_STR, __func__);
    return 0;
}

/**
 * This function opens the humi 
 * 
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_humi_bosch_bme680_open(void)
{
    int ret = 0;

   /* open pressure temperature humidity */
    ret = drv_baro_humi_temp_bosch_bme680_set_default_config(&bme680_ctx);
    if (unlikely(ret) != 0) {
        return ret;
    }
    ret  =  drv_baro_humi_temp_bosch_bme680_set_power_mode(
            &bme680_ctx, 
            DEV_POWER_ON);
    if (unlikely(ret) != 0) {
        return -1;
    }
    
    LOG("%s %s successfully \n", SENSOR_STR, __func__);
    return 0;

}

/**
 * This function closes the  humi
 * 
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_humi_bosch_bme680_close(void)
{
    int ret = 0;
    ret  = drv_baro_humi_temp_bosch_bme680_set_power_mode(
           &bme680_ctx,
           DEV_POWER_OFF);
    if (unlikely(ret) != 0) {
        return -1;
    }
    LOG("%s %s successfully \n", SENSOR_STR, __func__);
    return 0;
}

/**
 * This function reads the humi data and reports the data 
 * 
 * @param[in out]  buf   buffer for humi data
 * @param[in out]  len   length of data
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_humi_bosch_bme680_read(void *buf, size_t len)
{
    int                      ret = 0;
    size_t                   size = 0;
    humidity_data_t*         pdata = (barometer_data_t*)buf;
    struct bme680_field_data data;
    int8_t                   rslt = BME680_OK;
    if (buf == NULL) {
        return -1;
    }
    size = sizeof(humidity_data_t);
    if (len < size) {
        return -1;
    }
    
	ret = drv_baro_humi_temp_bosch_bme680_set_default_config(&bme680_ctx);
    if (unlikely(ret) != 0) {
        return -1;
    }

    rslt = bme680_get_sensor_data(&data, &g_bme680sensor);
	if (rslt != BME680_OK)
	    LOG("bme680_get_sensor_data failed,  rslt = %d \n", rslt);
  
  	printf("in humi:T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
	data.pressure / 100.0f, data.humidity / 1000.0f );
	/* Avoid using measurements from an unstable heating setup */
	if(data.status & BME680_GASM_VALID_MSK)
		printf(", G: %d ohms", data.gas_resistance);

	pdata->h = data.humidity / 100; /* ‰RH permille/per thousand */
    if (unlikely(ret) != 0) {
        return -1;
    }

    pdata->timestamp = aos_now_ms();
    
    return (int)size;
}

/**
 * This function writess the humi
 * 
 * @param[in out]  buf   buffer for written data
 * @param[in out]  len   length of data
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_humi_bosch_bme680_write(const void *buf, size_t len)
{
    (void)buf;
    (void)len;
    return 0;
}

/**
 * This function is for the humi ioctl
 * 
 * @param[in]  cmd   the ioctl command
 * @param[in]  arg   the correspondding parameter
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_humi_bosch_bme680_ioctl(int cmd, unsigned long arg)
{
    int ret = 0;
    
    switch(cmd){
        case SENSOR_IOCTL_ODR_SET:{
        }break;
        case SENSOR_IOCTL_SET_POWER:{
            ret = drv_baro_humi_temp_bosch_bme680_set_power_mode(&bme680_ctx, arg);
            if (unlikely(ret) != 0) {
                return -1;
            }
        }break;
        case SENSOR_IOCTL_GET_INFO:{ 
            /* fill the dev info here */
            dev_sensor_info_t *info = (dev_sensor_info_t *)arg;
            info->model = "BME680";
            info->range_max = 100;
            info->range_min = 0;
            info->unit = permillage;

        }break;
       
       default:break;
    }

    LOG("%s %s successfully \n", SENSOR_STR, __func__);
    return 0;
}

/**
 * This function is the ISR 
 * 
 * @return
 */
static void drv_humi_bosch_bme680_irq_handle(void)
{
    /* no handle so far */
}

/**
 * This function is for the humi initialization
 * 
 * @return  the operation status, 0 is OK, others is error
 */
int drv_humi_bosch_bme680_init(void)
{
    int          ret = 0;
    sensor_obj_t sensor;
	int8_t       rslt = BME680_OK;
    
	g_bme680sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
	g_bme680sensor.intf = BME680_I2C_INTF;
    g_bme680sensor.read = bme680_I2C_read;
    g_bme680sensor.write = bme680_I2C_write;
    g_bme680sensor.delay_ms = bme680_delay;
    g_bme680sensor.amb_temp = 25;

    /* fill the sensor obj parameters here */
    sensor.tag = TAG_DEV_HUMI;
    sensor.path = dev_humi_path;
    sensor.io_port = I2C_PORT;
    sensor.open = drv_humi_bosch_bme680_open;
    sensor.close = drv_humi_bosch_bme680_close;
    sensor.read = drv_humi_bosch_bme680_read;
    sensor.write = drv_humi_bosch_bme680_write;
    sensor.ioctl = drv_humi_bosch_bme680_ioctl;
    sensor.irq_handle = drv_humi_bosch_bme680_irq_handle;

    ret = sensor_create_obj(&sensor);
    if (unlikely(ret) != 0) {
        return -1;
    }

    if (g_bme680flag == 0) {
        rslt = bme680_init(&g_bme680sensor);
	    if(rslt != BME680_OK)
	    {
	        LOG("bme680_init failed,  rslt = %d \n", rslt);
		    return -1;
	    }
        ret = drv_baro_humi_temp_bosch_bme680_set_default_config(&bme680_ctx);
        if (unlikely(ret) != 0) {
            return -1;
        }
        g_bme680flag = 1;
   }
    
    LOG("%s %s successfully \n", SENSOR_STR, __func__);
    return 0;
}

/**
 * This function opens the temp 
 * 
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_temp_bosch_bme680_open(void)
{
    int ret = 0;

   /* open pressure temperature humidity */
    ret = drv_baro_humi_temp_bosch_bme680_set_default_config(&bme680_ctx);
    if (unlikely(ret) != 0) {
        return ret;
    }
    ret  =  drv_baro_humi_temp_bosch_bme680_set_power_mode(
            &bme680_ctx,
            DEV_POWER_ON);
    if (unlikely(ret) != 0) {
        return -1;
    }
    
    LOG("%s %s successfully \n", SENSOR_STR, __func__);
    return 0;

}

/**
 * This function closes the  temp
 * 
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_temp_bosch_bme680_close(void)
{
    int ret = 0;
    ret  = drv_baro_humi_temp_bosch_bme680_set_power_mode(
           &bme680_ctx,
           DEV_POWER_OFF);
    if (unlikely(ret) != 0) {
        return -1;
    }
    LOG("%s %s successfully \n", SENSOR_STR, __func__);
    return 0;
}

/**
 * This function reads the temp data and reports the data 
 * 
 * @param[in out]  buf   buffer for temp data
 * @param[in out]  len   length of data
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_temp_bosch_bme680_read(void *buf, size_t len)
{
    int                      ret = 0;
    size_t                   size = 0;
    temperature_data_t*      pdata = (barometer_data_t*)buf;
    struct bme680_field_data data;
    int8_t                   rslt = BME680_OK;
    if (buf == NULL) {
        return -1;
    }
    size = sizeof(temperature_data_t);
    if (len < size) {
        return -1;
    }
    
	ret = drv_baro_humi_temp_bosch_bme680_set_default_config(&bme680_ctx);
    if (unlikely(ret) != 0) {
        return -1;
    }

    rslt = bme680_get_sensor_data(&data, &g_bme680sensor);
	if (rslt != BME680_OK)
	    LOG("bme680_get_sensor_data failed,  rslt = %d \n", rslt);
  
  	printf("in humi:T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
	data.pressure / 100.0f, data.humidity / 1000.0f );
	/* Avoid using measurements from an unstable heating setup */
	if(data.status & BME680_GASM_VALID_MSK)
		printf(", G: %d ohms", data.gas_resistance);

	pdata->t = data.temperature / 100; /* degC */
    if (unlikely(ret) != 0) {
        return -1;
    }

    pdata->timestamp = aos_now_ms();
    
    return (int)size;
}

/**
 * This function writess the temp
 * 
 * @param[in out]  buf   buffer for written data
 * @param[in out]  len   length of data
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_temp_bosch_bme680_write(const void *buf, size_t len)
{
    (void)buf;
    (void)len;
    return 0;
}

/**
 * This function is for the temp ioctl
 * 
 * @param[in]  cmd   the ioctl command
 * @param[in]  arg   the correspondding parameter
 * @return  the operation status, 0 is OK, others is error
 */
static int drv_temp_bosch_bme680_ioctl(int cmd, unsigned long arg)
{
    int ret = 0;
    
    switch(cmd){
        case SENSOR_IOCTL_ODR_SET:{
        }break;
        case SENSOR_IOCTL_SET_POWER:{
            ret = drv_baro_humi_temp_bosch_bme680_set_power_mode(&bme680_ctx, arg);
            if (unlikely(ret) != 0) {
                return -1;
            }
        }break;
        case SENSOR_IOCTL_GET_INFO:{ 
            /* fill the dev info here */
            dev_sensor_info_t *info = (dev_sensor_info_t *)arg;
            info->model = "BME680";
            info->range_max = 85;
            info->range_min = -40;
            info->unit = dCelsius;

        }break;
       
       default:break;
    }

    LOG("%s %s successfully \n", SENSOR_STR, __func__);
    return 0;
}

/**
 * This function is the ISR 
 * 
 * @return
 */
static void drv_temp_bosch_bme680_irq_handle(void)
{
    /* no handle so far */
}

/**
 * This function is for the temp initialization
 * 
 * @return  the operation status, 0 is OK, others is error
 */
int drv_temp_bosch_bme680_init(void)
{
    int          ret = 0;
    sensor_obj_t sensor;
	int8_t       rslt = BME680_OK;
    
	g_bme680sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
	g_bme680sensor.intf = BME680_I2C_INTF;
    g_bme680sensor.read = bme680_I2C_read;
    g_bme680sensor.write = bme680_I2C_write;
    g_bme680sensor.delay_ms = bme680_delay;
    g_bme680sensor.amb_temp = 25;

    /* fill the sensor obj parameters here */
    sensor.tag = TAG_DEV_TEMP;
    sensor.path = dev_temp_path;
    sensor.io_port = I2C_PORT;
    sensor.open = drv_temp_bosch_bme680_open;
    sensor.close = drv_temp_bosch_bme680_close;
    sensor.read = drv_temp_bosch_bme680_read;
    sensor.write = drv_temp_bosch_bme680_write;
    sensor.ioctl = drv_temp_bosch_bme680_ioctl;
    sensor.irq_handle = drv_temp_bosch_bme680_irq_handle;

    ret = sensor_create_obj(&sensor);
    if (unlikely(ret) != 0) {
        return -1;
    }

    if (g_bme680flag == 0) {
        rslt = bme680_init(&g_bme680sensor);
	    if(rslt != BME680_OK)
	    {
	        LOG("bme680_init failed,  rslt = %d \n", rslt);
		    return -1;
	    }
        ret = drv_baro_humi_temp_bosch_bme680_set_default_config(&bme680_ctx);
        if (unlikely(ret) != 0) {
            return -1;
        }
        g_bme680flag = 1;
   }
    
    LOG("%s %s successfully \n", SENSOR_STR, __func__);
    return 0;
}