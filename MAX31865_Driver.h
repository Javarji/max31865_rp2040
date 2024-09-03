#ifndef __MAX31865_DRIVER_H
#define __MAX31865_DRIVER_H

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "../modules/MATH_F.h"

// MAX31865 SPI options
// Register READ ADDRESS (DS Table 1)
#define MAX31865_CONFIG_REG_R 0x00
#define MAX31865_RTDMSB_REG_R 0x01
#define MAX31865_RTDLSB_REG_R 0x02
#define MAX31865_HFAULTMSB_REG_R 0x03
#define MAX31865_HFAULTLSB_REG_R 0x04
#define MAX31865_LFAULTMSB_REG_R 0x05
#define MAX31865_LFAULTLSB_REG_R 0x06
#define MAX31865_FAULTSTAT_REG_R 0x07

// Register WRITE ADDRESS (DS Table 1)
#define MAX31865_CONFIG_REG_W 0x80
#define MAX31865_HFAULTMSB_REG_W 0x83
#define MAX31865_HFAULTLSB_REG_W 0x84
#define MAX31865_LFAULTMSB_REG_W 0x85
#define MAX31865_LFAULTLSB_REG_W 0x86

// Configuration Register Definition (DS Table 2)
// D7 V_BIAS
#define MAX31865_CONFIG_BIAS 0x80
// D6 Conversion mode
#define MAX31865_CONFIG_MODEAUTO 0x40
#define MAX31865_CONFIG_MODEOFF 0x00
// D5 shot
#define MAX31865_CONFIG_1SHOT 0x20
// D4 3 2-4 wires
#define MAX31865_CONFIG_3WIRE 0x10
#define MAX31865_CONFIG_24WIRE 0x00
// D2 D3 Fault detection control bits
#define MAX31865_CONFIG_FAULTDET_01 0x04
#define MAX31865_CONFIG_FAULTDET_10 0x08
#define MAX31865_CONFIG_FAULTDET_11 0x0C
// D1 Fault Status Clear
#define MAX31865_CONFIG_FAULTSTAT 0x02
// D0 50-60 Hz
#define MAX31865_CONFIG_FILT50HZ 0x01
#define MAX31865_CONFIG_FILT60HZ 0x00

// Faults
#define MAX31865_FAULT_HIGHTHRESH 0x80
#define MAX31865_FAULT_LOWTHRESH 0x40
#define MAX31865_FAULT_REFINLOW 0x20
#define MAX31865_FAULT_REFINHIGH 0x10
#define MAX31865_FAULT_RTDINLOW 0x08
#define MAX31865_FAULT_OVUV 0x04

// Reference resistor
#define MAX_31865_R_REF 4301.0
#define MAX_31865_R_0 1000.0

void MAX31865_pico_gpio_settings(spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_MISO, uint MAX31865_PIN_SCK, uint MAX31865_PIN_MOSI, uint MAX31865_PIN_CS, uint MAX31865_BAUD_RATE);
void MAX31865_read_registers(uint8_t reg, uint8_t *buf, uint16_t len, spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS);
void MAX31865_read_conf(uint8_t *buf, spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS);
void MAX31865_write_register(uint8_t reg, uint8_t data, spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS);
void MAX31865_initialize(spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS);
void MAX31865_start_stop_VBias(spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS, bool start_VBias);
void MAX31865_read_ADC(uint16_t *ADC, spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS);
void MAX31865_ADC_to_R_RTD(uint16_t ADC, float *R_RTD);
float f_R(float T);
float MAX31865_R_RTD_to_T_C(float R_RTD);
double f_R_d(double T);
float MAX31865_read_T_C(spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS);
#endif