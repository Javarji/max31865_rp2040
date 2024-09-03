#include "MAX31865_Driver.h"

void MAX31865_pico_gpio_settings(spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_MISO, uint MAX31865_PIN_SCK, uint MAX31865_PIN_MOSI, uint MAX31865_PIN_CS, uint MAX31865_BAUD_RATE){
    // SPI initialisation. This example will use SPI at 500kHz.
    spi_init(MAX31865_SPI_PORT, MAX31865_BAUD_RATE);
    spi_set_format(MAX31865_SPI_PORT,  8, (spi_cpol_t)1, (spi_cpha_t)1, SPI_MSB_FIRST); // Changing phase and clock polarity is the only way to make it work
    gpio_set_function(MAX31865_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(MAX31865_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(MAX31865_PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_function(MAX31865_PIN_CS, GPIO_FUNC_SIO);
    gpio_set_dir(MAX31865_PIN_CS, GPIO_OUT);
    gpio_put(MAX31865_PIN_CS, 1);
}

void MAX31865_read_registers(uint8_t reg, uint8_t *buf, uint16_t len, spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS){
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.
    gpio_put(MAX31865_PIN_CS, 0);
    // spi_write_read_blocking(MAX31865_SPI_PORT, &reg, buf,1);
    spi_write_blocking(MAX31865_SPI_PORT, &reg, 1);
    spi_read_blocking(MAX31865_SPI_PORT, 0, buf, len);
    gpio_put(MAX31865_PIN_CS, 1);
    
}

void MAX31865_write_register(uint8_t reg, uint8_t data, spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS) {
    uint8_t buf[2];
    buf[0] = reg;  // remove read bit as this is a write
    buf[1] = data;
    gpio_put(MAX31865_PIN_CS, 0);
    spi_write_blocking(MAX31865_SPI_PORT, buf, 2);
    gpio_put(MAX31865_PIN_CS, 1);
    // sleep_ms(10);
}

void MAX31865_read_conf(uint8_t *buf, spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS){
    MAX31865_read_registers(MAX31865_CONFIG_REG_R, buf, 1, MAX31865_SPI_PORT, MAX31865_PIN_CS);
}
void MAX31865_initialize(spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS){
    uint8_t data = 0x00;
    data = MAX31865_CONFIG_FAULTSTAT | MAX31865_CONFIG_FILT50HZ;
    MAX31865_write_register(MAX31865_CONFIG_REG_W, data, MAX31865_SPI_PORT, MAX31865_PIN_CS);    
}

void MAX31865_start_stop_VBias(spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS, bool start_VBias){
    uint8_t data;
    MAX31865_read_conf(&data, MAX31865_SPI_PORT, MAX31865_PIN_CS);
    data = MAX31865_CONFIG_BIAS | data;
    if (start_VBias){
        MAX31865_write_register(MAX31865_CONFIG_REG_W, data, MAX31865_SPI_PORT, MAX31865_PIN_CS); // Start Bias Voltage
    }
    else{
        data = (data << 1) >> 1;
        MAX31865_write_register(MAX31865_CONFIG_REG_W, data, MAX31865_SPI_PORT, MAX31865_PIN_CS); // Stop Bias Voltage
    }
    

    
    // Waiting time > 50ms according to data sheet not possible in repeating timer. a Kalman filter is needed
}
void MAX31865_read_ADC(uint16_t *ADC, spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS){
    uint8_t data;
    uint8_t RTDMSB_REG = 0x00;
    uint8_t RTDLSB_REG = 0x00;

    // MAX31865_start_stop_VBias(MAX31865_SPI_PORT, MAX31865_PIN_CS, true);
    //sleep_ms(100); // Waiting time > 50ms according to data sheet

    MAX31865_read_conf(&data, MAX31865_SPI_PORT, MAX31865_PIN_CS);
    data = MAX31865_CONFIG_1SHOT | data;
    
    MAX31865_write_register(MAX31865_CONFIG_REG_W, data, MAX31865_SPI_PORT, MAX31865_PIN_CS); // Start 1-shot
    MAX31865_read_registers(MAX31865_RTDMSB_REG_R, &RTDMSB_REG, 1, MAX31865_SPI_PORT, MAX31865_PIN_CS);
    MAX31865_read_registers(MAX31865_RTDLSB_REG_R, &RTDLSB_REG, 1, MAX31865_SPI_PORT, MAX31865_PIN_CS);
    *ADC = (uint16_t)RTDMSB_REG << 7 | (uint16_t)RTDLSB_REG >> 1;   
    // MAX31865_start_stop_VBias(MAX31865_SPI_PORT, MAX31865_PIN_CS, false);
}

void MAX31865_ADC_to_R_RTD(uint16_t ADC, float *R_RTD){
    *R_RTD = (ADC * MAX_31865_R_REF)/pow(2,15);
}

float f_R(float T){
    // Based on: Resistance_Table_DIN_EN_60751_1000_Ohm_02-2001_EN
    double A =  3.9083*pow(10,-3);
    double B = -5.7750*pow(10,-7);
    long double C = -4.1830*pow(10,-12);
    
    if (T>=0){
        return (float)(MAX_31865_R_0 * (1 + A*T + B*pow(T,2)));
    }
    else{
        return (float)(MAX_31865_R_0 * (1 + A*T + B*pow(T,2) + C*(long double)(T-100)*pow(T,3)));
    }
}

double f_R_d(double T){
    // Based on: Resistance_Table_DIN_EN_60751_1000_Ohm_02-2001_EN
    double A =  3.9083*pow(10,-3);
    double B = -5.7750*pow(10,-7);
    long double C = -4.1830*pow(10,-12);
    
    if (T>=0){
        return (MAX_31865_R_0 * (1 + A*T + B*pow(T,2)));
    }
    else{
        return (MAX_31865_R_0 * (1 + A*T + B*pow(T,2) + C*(long double)(T-100)*pow(T,3)));
    }
}

float MAX31865_R_RTD_to_T_C(float R_RTD){
    double T_avg;

    T_avg = bisection(f_R_d, -50.0, 300.0, (double)R_RTD, 0.0001);    

    return (float)T_avg;
}

float MAX31865_read_T_C(spi_inst_t *MAX31865_SPI_PORT, uint MAX31865_PIN_CS){
    uint16_t ADC;
    float R_RTD;
    float T_PT1000;
    MAX31865_read_ADC(&ADC, MAX31865_SPI_PORT, MAX31865_PIN_CS);
    MAX31865_ADC_to_R_RTD(ADC, &R_RTD);
    T_PT1000 = MAX31865_R_RTD_to_T_C(R_RTD);
    // printf("%s:%lu:ADC %.lu\tR_RTD %.1f\tT_PT1000 %.3f\n",__func__,__LINE__, ADC, R_RTD, T_PT1000);
    return T_PT1000;
}