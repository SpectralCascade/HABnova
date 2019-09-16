# 1 "BME280_driver/bme280.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "BME280_driver/bme280.c" 2
# 51 "BME280_driver/bme280.c"
# 1 "BME280_driver/bme280.h" 1
# 65 "BME280_driver/bme280.h"
# 1 "BME280_driver/bme280_defs.h" 1
# 66 "BME280_driver/bme280_defs.h"
# 1 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\stdint.h" 1 3



# 1 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\musl_xc8.h" 1 3
# 4 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\stdint.h" 2 3
# 22 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\stdint.h" 3
# 1 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 135 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uintptr_t;
# 150 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long intptr_t;
# 166 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;




typedef __int24 int24_t;




typedef long int32_t;





typedef long long int64_t;
# 196 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long long intmax_t;





typedef unsigned char uint8_t;




typedef unsigned short uint16_t;




typedef __uint24 uint24_t;




typedef unsigned long uint32_t;





typedef unsigned long long uint64_t;
# 237 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long long uintmax_t;
# 22 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\stdint.h" 2 3


typedef int8_t int_fast8_t;

typedef int64_t int_fast64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;

typedef int32_t int_least32_t;

typedef int64_t int_least64_t;


typedef uint8_t uint_fast8_t;

typedef uint64_t uint_fast64_t;


typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;

typedef uint32_t uint_least32_t;

typedef uint64_t uint_least64_t;
# 155 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\stdint.h" 3
# 1 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\bits/stdint.h" 1 3
typedef int32_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint32_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 155 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\stdint.h" 2 3
# 66 "BME280_driver/bme280_defs.h" 2

# 1 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\stddef.h" 1 3
# 19 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\stddef.h" 3
# 1 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 22 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long int wchar_t;
# 127 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned size_t;
# 140 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long ptrdiff_t;
# 19 "K:\\Programs\\MPLABX\\XC8 Compiler\\pic\\include\\c99\\stddef.h" 2 3
# 67 "BME280_driver/bme280_defs.h" 2
# 243 "BME280_driver/bme280_defs.h"
enum bme280_intf {

    BME280_SPI_INTF,


    BME280_I2C_INTF
};




typedef int8_t (*bme280_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*bme280_delay_fptr_t)(uint32_t period);




struct bme280_calib_data
{





    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
    int32_t t_fine;


};
# 307 "BME280_driver/bme280_defs.h"
struct bme280_data
{

    uint32_t pressure;


    int32_t temperature;


    uint32_t humidity;
};






struct bme280_uncomp_data
{

    uint32_t pressure;


    uint32_t temperature;


    uint32_t humidity;
};





struct bme280_settings
{

    uint8_t osr_p;


    uint8_t osr_t;


    uint8_t osr_h;


    uint8_t filter;


    uint8_t standby_time;
};




struct bme280_dev
{

    uint8_t chip_id;


    uint8_t dev_id;


    enum bme280_intf intf;


    bme280_com_fptr_t read;


    bme280_com_fptr_t write;


    bme280_delay_fptr_t delay_ms;


    struct bme280_calib_data calib_data;


    struct bme280_settings settings;
};
# 65 "BME280_driver/bme280.h" 2
# 76 "BME280_driver/bme280.h"
int8_t bme280_init(struct bme280_dev *dev);
# 91 "BME280_driver/bme280.h"
int8_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bme280_dev *dev);
# 104 "BME280_driver/bme280.h"
int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bme280_dev *dev);
# 129 "BME280_driver/bme280.h"
int8_t bme280_set_sensor_settings(uint8_t desired_settings, const struct bme280_dev *dev);
# 140 "BME280_driver/bme280.h"
int8_t bme280_get_sensor_settings(struct bme280_dev *dev);
# 157 "BME280_driver/bme280.h"
int8_t bme280_set_sensor_mode(uint8_t sensor_mode, const struct bme280_dev *dev);
# 174 "BME280_driver/bme280.h"
int8_t bme280_get_sensor_mode(uint8_t *sensor_mode, const struct bme280_dev *dev);
# 184 "BME280_driver/bme280.h"
int8_t bme280_soft_reset(const struct bme280_dev *dev);
# 207 "BME280_driver/bme280.h"
int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev);
# 217 "BME280_driver/bme280.h"
void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data);
# 235 "BME280_driver/bme280.h"
int8_t bme280_compensate_data(uint8_t sensor_comp,
                              const struct bme280_uncomp_data *uncomp_data,
                              struct bme280_data *comp_data,
                              struct bme280_calib_data *calib_data);
# 51 "BME280_driver/bme280.c" 2
# 68 "BME280_driver/bme280.c"
static int8_t put_device_to_sleep(const struct bme280_dev *dev);
# 79 "BME280_driver/bme280.c"
static int8_t write_power_mode(uint8_t sensor_mode, const struct bme280_dev *dev);
# 90 "BME280_driver/bme280.c"
static int8_t null_ptr_check(const struct bme280_dev *dev);
# 103 "BME280_driver/bme280.c"
static void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len);
# 114 "BME280_driver/bme280.c"
static int8_t get_calib_data(struct bme280_dev *dev);
# 123 "BME280_driver/bme280.c"
static void parse_temp_press_calib_data(const uint8_t *reg_data, struct bme280_dev *dev);
# 132 "BME280_driver/bme280.c"
static void parse_humidity_calib_data(const uint8_t *reg_data, struct bme280_dev *dev);
# 187 "BME280_driver/bme280.c"
static int32_t compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
                                      struct bme280_calib_data *calib_data);
# 200 "BME280_driver/bme280.c"
static uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
                                    const struct bme280_calib_data *calib_data);
# 213 "BME280_driver/bme280.c"
static uint32_t compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
                                    const struct bme280_calib_data *calib_data);
# 231 "BME280_driver/bme280.c"
static uint8_t are_settings_changed(uint8_t sub_settings, uint8_t desired_settings);
# 241 "BME280_driver/bme280.c"
static int8_t set_osr_humidity_settings(const struct bme280_settings *settings, const struct bme280_dev *dev);
# 254 "BME280_driver/bme280.c"
static int8_t set_osr_settings(uint8_t desired_settings,
                               const struct bme280_settings *settings,
                               const struct bme280_dev *dev);
# 269 "BME280_driver/bme280.c"
static int8_t set_osr_press_temp_settings(uint8_t desired_settings,
                                          const struct bme280_settings *settings,
                                          const struct bme280_dev *dev);
# 281 "BME280_driver/bme280.c"
static void fill_osr_press_settings(uint8_t *reg_data, const struct bme280_settings *settings);
# 291 "BME280_driver/bme280.c"
static void fill_osr_temp_settings(uint8_t *reg_data, const struct bme280_settings *settings);
# 304 "BME280_driver/bme280.c"
static int8_t set_filter_standby_settings(uint8_t desired_settings,
                                          const struct bme280_settings *settings,
                                          const struct bme280_dev *dev);
# 316 "BME280_driver/bme280.c"
static void fill_filter_settings(uint8_t *reg_data, const struct bme280_settings *settings);
# 326 "BME280_driver/bme280.c"
static void fill_standby_settings(uint8_t *reg_data, const struct bme280_settings *settings);
# 336 "BME280_driver/bme280.c"
static void parse_device_settings(const uint8_t *reg_data, struct bme280_settings *settings);
# 349 "BME280_driver/bme280.c"
static int8_t reload_device_settings(const struct bme280_settings *settings, const struct bme280_dev *dev);







int8_t bme280_init(struct bme280_dev *dev)
{
    int8_t rslt;


    uint8_t try_count = 5;
    uint8_t chip_id = 0;


    rslt = null_ptr_check(dev);


    if (rslt == 0)
    {
        while (try_count)
        {

            rslt = bme280_get_regs(0xD0, &chip_id, 1, dev);

            if ((rslt == 0) && (chip_id == 0x60))
            {
                dev->chip_id = chip_id;


                rslt = bme280_soft_reset(dev);
                if (rslt == 0)
                {

                    rslt = get_calib_data(dev);
                }
                break;
            }


            dev->delay_ms(1);
            --try_count;
        }


        if (!try_count)
        {
            rslt = -2;
        }
    }

    return rslt;
}




int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bme280_dev *dev)
{
    int8_t rslt;


    rslt = null_ptr_check(dev);


    if (rslt == 0)
    {

        if (dev->intf != BME280_I2C_INTF)
        {
            reg_addr = reg_addr | 0x80;
        }


        rslt = dev->read(dev->dev_id, reg_addr, reg_data, len);


        if (rslt != 0)
        {
            rslt = -4;
        }
    }

    return rslt;
}





int8_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t temp_buff[20];

    if (len > 10)
    {
        len = 10;
    }
    uint16_t temp_len;
    uint8_t reg_addr_cnt;


    rslt = null_ptr_check(dev);


    if ((rslt == 0) && (reg_addr != ((void*)0)) && (reg_data != ((void*)0)))
    {
        if (len != 0)
        {
            temp_buff[0] = reg_data[0];


            if (dev->intf != BME280_I2C_INTF)
            {
                for (reg_addr_cnt = 0; reg_addr_cnt < len; reg_addr_cnt++)
                {
                    reg_addr[reg_addr_cnt] = reg_addr[reg_addr_cnt] & 0x7F;
                }
            }


            if (len > 1)
            {



                interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
                temp_len = ((len * 2) - 1);
            }
            else
            {
                temp_len = len;
            }
            rslt = dev->write(dev->dev_id, reg_addr[0], temp_buff, temp_len);


            if (rslt != 0)
            {
                rslt = -4;
            }
        }
        else
        {
            rslt = -3;
        }
    }
    else
    {
        rslt = -1;
    }

    return rslt;
}





int8_t bme280_set_sensor_settings(uint8_t desired_settings, const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t sensor_mode;


    rslt = null_ptr_check(dev);


    if (rslt == 0)
    {
        rslt = bme280_get_sensor_mode(&sensor_mode, dev);
        if ((rslt == 0) && (sensor_mode != 0x00))
        {
            rslt = put_device_to_sleep(dev);
        }
        if (rslt == 0)
        {



            if (are_settings_changed(0x07, desired_settings))
            {
                rslt = set_osr_settings(desired_settings, &dev->settings, dev);
            }




            if ((rslt == 0) && are_settings_changed(0x18, desired_settings))
            {
                rslt = set_filter_standby_settings(desired_settings, &dev->settings, dev);
            }
        }
    }

    return rslt;
}





int8_t bme280_get_sensor_settings(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data[4];


    rslt = null_ptr_check(dev);


    if (rslt == 0)
    {
        rslt = bme280_get_regs(0xF2, reg_data, 4, dev);
        if (rslt == 0)
        {
            parse_device_settings(reg_data, &dev->settings);
        }
    }

    return rslt;
}




int8_t bme280_set_sensor_mode(uint8_t sensor_mode, const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t last_set_mode;


    rslt = null_ptr_check(dev);
    if (rslt == 0)
    {
        rslt = bme280_get_sensor_mode(&last_set_mode, dev);




        if ((rslt == 0) && (last_set_mode != 0x00))
        {
            rslt = put_device_to_sleep(dev);
        }


        if (rslt == 0)
        {
            rslt = write_power_mode(sensor_mode, dev);
        }
    }

    return rslt;
}




int8_t bme280_get_sensor_mode(uint8_t *sensor_mode, const struct bme280_dev *dev)
{
    int8_t rslt;


    rslt = null_ptr_check(dev);
    if (rslt == 0)
    {

        rslt = bme280_get_regs(0xF4, sensor_mode, 1, dev);


        *sensor_mode = (*sensor_mode & ( 0x03));
    }

    return rslt;
}




int8_t bme280_soft_reset(const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = 0xE0;


    uint8_t soft_rst_cmd = 0xB6;


    rslt = null_ptr_check(dev);


    if (rslt == 0)
    {

        rslt = bme280_set_regs(&reg_addr, &soft_rst_cmd, 1, dev);


        dev->delay_ms(2);
    }

    return rslt;
}






int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev)
{
    int8_t rslt;




    uint8_t reg_data[8] = { 0 };
    struct bme280_uncomp_data uncomp_data = { 0 };


    rslt = null_ptr_check(dev);
    if ((rslt == 0) && (comp_data != ((void*)0)))
    {

        rslt = bme280_get_regs(0xF7, reg_data, 8, dev);
        if (rslt == 0)
        {

            bme280_parse_sensor_data(reg_data, &uncomp_data);




            rslt = bme280_compensate_data(sensor_comp, &uncomp_data, comp_data, &dev->calib_data);
        }
    }
    else
    {
        rslt = -1;
    }

    return rslt;
}





void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data)
{

    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;


    data_msb = (uint32_t)reg_data[0] << 12;
    data_lsb = (uint32_t)reg_data[1] << 4;
    data_xlsb = (uint32_t)reg_data[2] >> 4;
    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;


    data_msb = (uint32_t)reg_data[3] << 12;
    data_lsb = (uint32_t)reg_data[4] << 4;
    data_xlsb = (uint32_t)reg_data[5] >> 4;
    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;


    data_lsb = (uint32_t)reg_data[6] << 8;
    data_msb = (uint32_t)reg_data[7];
    uncomp_data->humidity = data_msb | data_lsb;
}






int8_t bme280_compensate_data(uint8_t sensor_comp,
                              const struct bme280_uncomp_data *uncomp_data,
                              struct bme280_data *comp_data,
                              struct bme280_calib_data *calib_data)
{
    int8_t rslt = 0;

    if ((uncomp_data != ((void*)0)) && (comp_data != ((void*)0)) && (calib_data != ((void*)0)))
    {

        comp_data->temperature = 0;
        comp_data->pressure = 0;
        comp_data->humidity = 0;


        if (sensor_comp & (1 | 1 << 1 | 1 << 2))
        {

            comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
        }
        if (sensor_comp & 1)
        {

            comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
        }
        if (sensor_comp & 1 << 2)
        {


            comp_data->humidity = compensate_humidity(uncomp_data, calib_data);




        }
    }
    else
    {
        rslt = -1;
    }

    return rslt;
}





static int8_t set_osr_settings(uint8_t desired_settings,
                               const struct bme280_settings *settings,
                               const struct bme280_dev *dev)
{
    int8_t rslt = 1;

    if (desired_settings & 1 << 2)
    {
        rslt = set_osr_humidity_settings(settings, dev);
    }
    if (desired_settings & (1 | 1 << 1))
    {
        rslt = set_osr_press_temp_settings(desired_settings, settings, dev);
    }

    return rslt;
}




static int8_t set_osr_humidity_settings(const struct bme280_settings *settings, const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t ctrl_hum;
    uint8_t ctrl_meas;
    uint8_t reg_addr = 0xF2;

    ctrl_hum = settings->osr_h & 0x07;


    rslt = bme280_set_regs(&reg_addr, &ctrl_hum, 1, dev);




    if (rslt == 0)
    {
        reg_addr = 0xF4;
        rslt = bme280_get_regs(reg_addr, &ctrl_meas, 1, dev);
        if (rslt == 0)
        {
            rslt = bme280_set_regs(&reg_addr, &ctrl_meas, 1, dev);
        }
    }

    return rslt;
}





static int8_t set_osr_press_temp_settings(uint8_t desired_settings,
                                          const struct bme280_settings *settings,
                                          const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = 0xF4;
    uint8_t reg_data;

    rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);
    if (rslt == 0)
    {
        if (desired_settings & 1)
        {
            fill_osr_press_settings(&reg_data, settings);
        }
        if (desired_settings & 1 << 1)
        {
            fill_osr_temp_settings(&reg_data, settings);
        }


        rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
    }

    return rslt;
}





static int8_t set_filter_standby_settings(uint8_t desired_settings,
                                          const struct bme280_settings *settings,
                                          const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = 0xF5;
    uint8_t reg_data;

    rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);
    if (rslt == 0)
    {
        if (desired_settings & 1 << 3)
        {
            fill_filter_settings(&reg_data, settings);
        }
        if (desired_settings & 1 << 4)
        {
            fill_standby_settings(&reg_data, settings);
        }


        rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
    }

    return rslt;
}





static void fill_filter_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
    *reg_data = ((*reg_data & ~( 0x1C)) | ((settings->filter << 0x02) & 0x1C));
}





static void fill_standby_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
    *reg_data = ((*reg_data & ~( 0xE0)) | ((settings->standby_time << 0x05) & 0xE0));
}





static void fill_osr_press_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
    *reg_data = ((*reg_data & ~( 0x1C)) | ((settings->osr_p << 0x02) & 0x1C));
}





static void fill_osr_temp_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
    *reg_data = ((*reg_data & ~( 0xE0)) | ((settings->osr_t << 0x05) & 0xE0));
}






static void parse_device_settings(const uint8_t *reg_data, struct bme280_settings *settings)
{
    settings->osr_h = (reg_data[0] & ( 0x07));
    settings->osr_p = ((reg_data[2] & ( 0x1C)) >> ( 0x02));
    settings->osr_t = ((reg_data[2] & ( 0xE0)) >> ( 0x05));
    settings->filter = ((reg_data[3] & ( 0x1C)) >> ( 0x02));
    settings->standby_time = ((reg_data[3] & ( 0xE0)) >> ( 0x05));
}




static int8_t write_power_mode(uint8_t sensor_mode, const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = 0xF4;


    uint8_t sensor_mode_reg_val;


    rslt = bme280_get_regs(reg_addr, &sensor_mode_reg_val, 1, dev);


    if (rslt == 0)
    {
        sensor_mode_reg_val = ((sensor_mode_reg_val & ~( 0x03)) | (sensor_mode & 0x03));


        rslt = bme280_set_regs(&reg_addr, &sensor_mode_reg_val, 1, dev);
    }

    return rslt;
}




static int8_t put_device_to_sleep(const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data[4];
    struct bme280_settings settings;

    rslt = bme280_get_regs(0xF2, reg_data, 4, dev);
    if (rslt == 0)
    {
        parse_device_settings(reg_data, &settings);
        rslt = bme280_soft_reset(dev);
        if (rslt == 0)
        {
            rslt = reload_device_settings(&settings, dev);
        }
    }

    return rslt;
}





static int8_t reload_device_settings(const struct bme280_settings *settings, const struct bme280_dev *dev)
{
    int8_t rslt;

    rslt = set_osr_settings(0x1F, settings, dev);
    if (rslt == 0)
    {
        rslt = set_filter_standby_settings(0x1F, settings, dev);
    }

    return rslt;
}
# 1125 "BME280_driver/bme280.c"
static int32_t compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
                                      struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    var1 = (int32_t)((uncomp_data->temperature / 8) - ((int32_t)calib_data->dig_T1 * 2));
    var1 = (var1 * ((int32_t)calib_data->dig_T2)) / 2048;
    var2 = (int32_t)((uncomp_data->temperature / 16) - ((int32_t)calib_data->dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data->dig_T3)) / 16384;
    calib_data->t_fine = var1 + var2;
    temperature = (calib_data->t_fine * 5 + 128) / 256;
    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}
# 1208 "BME280_driver/bme280.c"
static uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
                                    const struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    uint32_t var5;
    uint32_t pressure;
    uint32_t pressure_min = 30000;
    uint32_t pressure_max = 110000;

    var1 = (((int32_t)calib_data->t_fine) / 2) - (int32_t)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_data->dig_P6);
    var2 = var2 + ((var1 * ((int32_t)calib_data->dig_P5)) * 2);
    var2 = (var2 / 4) + (((int32_t)calib_data->dig_P4) * 65536);
    var3 = (calib_data->dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t)calib_data->dig_P2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t)calib_data->dig_P1)) / 32768;


    if (var1)
    {
        var5 = (uint32_t)((uint32_t)1048576) - uncomp_data->pressure;
        pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;
        if (pressure < 0x80000000)
        {
            pressure = (pressure << 1) / ((uint32_t)var1);
        }
        else
        {
            pressure = (pressure / (uint32_t)var1) * 2;
        }
        var1 = (((int32_t)calib_data->dig_P9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
        var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_data->dig_P8)) / 8192;
        pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_data->dig_P7) / 16));
        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}






static uint32_t compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
                                    const struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;
    uint32_t humidity_max = 102400;

    var1 = calib_data->t_fine - ((int32_t)76800);
    var2 = (int32_t)(uncomp_data->humidity * 16384);
    var3 = (int32_t)(((int32_t)calib_data->dig_H4) * 1048576);
    var4 = ((int32_t)calib_data->dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calib_data->dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)calib_data->dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calib_data->dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calib_data->dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);
    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }

    return humidity;
}






static int8_t get_calib_data(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = 0x88;


    uint8_t calib_data[26] = { 0 };


    rslt = bme280_get_regs(reg_addr, calib_data, 26, dev);
    if (rslt == 0)
    {



        parse_temp_press_calib_data(calib_data, dev);
        reg_addr = 0xE1;


        rslt = bme280_get_regs(reg_addr, calib_data, 7, dev);
        if (rslt == 0)
        {



            parse_humidity_calib_data(calib_data, dev);
        }
    }

    return rslt;
}





static void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len)
{
    uint8_t index;

    for (index = 1; index < len; index++)
    {
        temp_buff[(index * 2) - 1] = reg_addr[index];
        temp_buff[index * 2] = reg_data[index];
    }
}





static void parse_temp_press_calib_data(const uint8_t *reg_data, struct bme280_dev *dev)
{
    struct bme280_calib_data *calib_data = &dev->calib_data;

    calib_data->dig_T1 = (((uint16_t)reg_data[1] << 8) | (uint16_t)reg_data[0]);
    calib_data->dig_T2 = (int16_t)(((uint16_t)reg_data[3] << 8) | (uint16_t)reg_data[2]);
    calib_data->dig_T3 = (int16_t)(((uint16_t)reg_data[5] << 8) | (uint16_t)reg_data[4]);
    calib_data->dig_P1 = (((uint16_t)reg_data[7] << 8) | (uint16_t)reg_data[6]);
    calib_data->dig_P2 = (int16_t)(((uint16_t)reg_data[9] << 8) | (uint16_t)reg_data[8]);
    calib_data->dig_P3 = (int16_t)(((uint16_t)reg_data[11] << 8) | (uint16_t)reg_data[10]);
    calib_data->dig_P4 = (int16_t)(((uint16_t)reg_data[13] << 8) | (uint16_t)reg_data[12]);
    calib_data->dig_P5 = (int16_t)(((uint16_t)reg_data[15] << 8) | (uint16_t)reg_data[14]);
    calib_data->dig_P6 = (int16_t)(((uint16_t)reg_data[17] << 8) | (uint16_t)reg_data[16]);
    calib_data->dig_P7 = (int16_t)(((uint16_t)reg_data[19] << 8) | (uint16_t)reg_data[18]);
    calib_data->dig_P8 = (int16_t)(((uint16_t)reg_data[21] << 8) | (uint16_t)reg_data[20]);
    calib_data->dig_P9 = (int16_t)(((uint16_t)reg_data[23] << 8) | (uint16_t)reg_data[22]);
    calib_data->dig_H1 = reg_data[25];
}





static void parse_humidity_calib_data(const uint8_t *reg_data, struct bme280_dev *dev)
{
    struct bme280_calib_data *calib_data = &dev->calib_data;
    int16_t dig_H4_lsb;
    int16_t dig_H4_msb;
    int16_t dig_H5_lsb;
    int16_t dig_H5_msb;

    calib_data->dig_H2 = (int16_t)(((uint16_t)reg_data[1] << 8) | (uint16_t)reg_data[0]);
    calib_data->dig_H3 = reg_data[2];
    dig_H4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    dig_H4_lsb = (int16_t)(reg_data[4] & 0x0F);
    calib_data->dig_H4 = dig_H4_msb | dig_H4_lsb;
    dig_H5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    dig_H5_lsb = (int16_t)(reg_data[4] >> 4);
    calib_data->dig_H5 = dig_H5_msb | dig_H5_lsb;
    calib_data->dig_H6 = (int8_t)reg_data[6];
}





static uint8_t are_settings_changed(uint8_t sub_settings, uint8_t desired_settings)
{
    uint8_t settings_changed = 0;

    if (sub_settings & desired_settings)
    {

        settings_changed = 1;
    }
    else
    {

        settings_changed = 0;
    }

    return settings_changed;
}





static int8_t null_ptr_check(const struct bme280_dev *dev)
{
    int8_t rslt;

    if ((dev == ((void*)0)) || (dev->read == ((void*)0)) || (dev->write == ((void*)0)) || (dev->delay_ms == ((void*)0)))
    {

        rslt = -1;
    }
    else
    {

        rslt = 0;
    }

    return rslt;
}
