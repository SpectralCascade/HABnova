// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_I2C_USER_H
#define	XC_HEADER_I2C_USER_H
#endif	/* XC_HEADER_I2C_USER_H */

#ifndef I2C_FUNCS
#define I2C_FUNCS
#endif // I2C_FUNCS

#ifdef I2C_FUNCS
#ifndef BME280_SENSOR
#define BME280_SENSOR
#endif // BME280_SENSOR
#endif // I2C_FUNCS

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>

typedef enum
{
    I2C_ERROR_NOTHING,
    I2C_ERROR_WRITE_NO_ACK,
    I2C_ERROR_COLISION
} I2C_ERROR;

#ifdef I2C_FUNCS

void I2C_Wait_SSPIF(void);

void I2C_Init(void);
void I2C_WriteByte(uint8_t data);
uint8_t I2C_ReadByte(uint8_t ackbit);
void I2C_Stop(void);
void I2C_UserAlert(I2C_ERROR status);

#ifdef BME280_SENSOR
extern struct bme280_dev EnvSensor;

void BME280_Init(void);
int8_t ReadEnvSensor(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t WriteEnvSensor(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
#endif // BME280_SENSOR

#endif // I2C_FUNCS
