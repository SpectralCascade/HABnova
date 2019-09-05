#include <stdio.h>
#include "xc.h"
#include "BME280_driver/bme280.h"
#include "bme280_i2c.h"
#include "timing.h"
#include "debug.h"

#ifdef I2C_FUNCS

#define I2C_TIMEOUT 3000

#define I2C_WAIT_SSPIF()                                                        \
    await_timeout(PIR1bits.SSP1IF != 0, I2C_TIMEOUT, FlashError);               \
    PIR1bits.SSP1IF = 0;

void I2C_Init(void)
{
    SSPCLKPPS = 0x0E;   //RB6->MSSP:SCL;
    SSPDATPPS = 0x0C;   //RB4->MSSP:SDA;
    RB6PPS = 0x10;   //RB6->MSSP:SCL;
    RB4PPS = 0x11;   //RB4->MSSP:SDA;

    SSP1STAT = 0x80; // standard
    SSP1CON1 = 0x28;
    SSP1CON2 = 0x00;
    SSP1ADD = 0x4f;
    
    SSP1CON1bits.SSPEN =  1;
}

void I2C_WriteByte(uint8_t data)
{
    SSP1BUF = data; // write data
    I2C_WAIT_SSPIF();
    
    // ACK check
    if(SSP1CON2bits.ACKSTAT!=0){
        I2C_UserAlert(I2C_ERROR_WRITE_NO_ACK); // NACK
    }
    
    return;
}

uint8_t I2C_ReadByte(uint8_t ackbit)
{
    uint8_t rcvdata;
    
    SSP1CON2bits.RCEN = 1; // Receive enable
    I2C_WAIT_SSPIF();
    
    rcvdata = SSP1BUF;
    
    // send ack or nak
    SSP1CON2bits.ACKDT=ackbit;
    SSP1CON2bits.ACKEN = 1;
    I2C_WAIT_SSPIF();    
    
    return (rcvdata);
}

void I2C_Stop(void)
{
    SSP1CON2bits.PEN = 1;
    I2C_WAIT_SSPIF();
}

void I2C_UserAlert(I2C_ERROR status)
{
    I2C_Stop();
}

#ifdef BME280_SENSOR

struct bme280_dev EnvSensor;

void BME280_Init()
{
    I2C_Init();

    int8_t rslt = BME280_OK;

    EnvSensor.dev_id = BME280_I2C_ADDR_PRIM;
    EnvSensor.intf = BME280_I2C_INTF;
    EnvSensor.read = ReadEnvSensor;
    EnvSensor.write = WriteEnvSensor;
    EnvSensor.delay_ms = Sleep;

    rslt = bme280_init(&EnvSensor);
    
    if (rslt == BME280_OK){
        uint8_t settings_sel;

        /* Recommended mode of operation: Indoor navigation */
        EnvSensor.settings.osr_h = BME280_OVERSAMPLING_1X;
        EnvSensor.settings.osr_p = BME280_OVERSAMPLING_16X;
        EnvSensor.settings.osr_t = BME280_OVERSAMPLING_2X;
        EnvSensor.settings.filter = BME280_FILTER_COEFF_16;
        EnvSensor.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

        settings_sel = BME280_OSR_PRESS_SEL;
        settings_sel |= BME280_OSR_TEMP_SEL;
        settings_sel |= BME280_OSR_HUM_SEL;
        settings_sel |= BME280_STANDBY_SEL;
        settings_sel |= BME280_FILTER_SEL;
        rslt = bme280_set_sensor_settings(settings_sel, &EnvSensor);
        rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &EnvSensor);
        
        if (rslt == BME280_OK)
        {
            return;
        }
    }

    FlashError();
}

int8_t ReadEnvSensor(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0;
 
    PIR1bits.SSP1IF = 0; // clear flag
    
    // set start condition
    SSP1CON2bits.SEN = 1;
    I2C_WAIT_SSPIF();
    
    // send device address
    I2C_WriteByte(dev_id << 1); // write
    
    // send data
    I2C_WriteByte(reg_addr);
    
    // set repeat start condition
    SSP1CON2bits.RSEN = 1;
    I2C_WAIT_SSPIF();
    
    //send device address
    I2C_WriteByte((dev_id << 1) | 0x01); // read
    
    // receive data
    for(int i = 0; i < len; i++){
        if(i == len - 1){
            reg_data[i] = I2C_ReadByte(1);  // send nack
        }
        else{
            reg_data[i]=I2C_ReadByte(0);  // send ack
        }
    }
    
    // set stop condition
    I2C_Stop();
     
    return rslt;
}
 
int8_t WriteEnvSensor(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0;
    
    PIR1bits.SSP1IF = 0; // clear flag
    
    // set start condition
    SSP1CON2bits.SEN = 1;
    I2C_WAIT_SSPIF();
    
    // send device address
    I2C_WriteByte(dev_id<<1); // write
    
    // send data
    I2C_WriteByte(reg_addr);

    for(int i = 0; i < len; i++){
        I2C_WriteByte(reg_data[i]);
    }
    
    I2C_Stop();
    
    return rslt;
}

#endif // BME280_SENSOR

#endif // I2C_FUNCS
