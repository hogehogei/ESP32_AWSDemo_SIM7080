#include <bme280/bme280.hpp>

#include "esp_log.h"
#include "driver/i2c.h"


// 
// define static variables
//

//
// define static const variables
//
const char sk_BME280_Tag[] = "BME280";

#define I2C_MASTER_SCL_IO           CONFIG_BME280_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_BME280_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                                 /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                            /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                                 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                                 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// BME280 temperature compensation registers "dig_T*"
static const uint8_t sk_bme280_temperature_comp_reg[] = {
    0x88, 0x89, 
    0x8A, 0x8B,
    0x8C, 0x8D
};
#define I2C_BME280_DIGT_REG_NUM ((int)sizeof(sk_bme280_temperature_comp_reg))

// BME280 pressure compensation registers "dig_P*"
static const uint8_t sk_bme280_pressure_comp_reg[] = {
    0x8E, 0x8F,
    0x90, 0x91,
    0x92, 0x93,
    0x94, 0x95,
    0x96, 0x97,
    0x98, 0x99,
    0x9A, 0x9B,
    0x9C, 0x9D,
    0x9E, 0x9F,
};
#define I2C_BME280_DIGP_REG_NUM ((int)sizeof(sk_bme280_pressure_comp_reg))

// BME280 humidity compensation registers "dig_H*"
static const uint8_t sk_bme280_humidity_comp_reg_hi[] = {
    0xA1
};
static const uint8_t sk_bme280_humidity_comp_reg_lo[] = {
    0xE1, 0xE2,
    0xE3,
    0xE4, 0xE5, 0xE6,
    0xE7,
};
#define I2C_BME280_DIGH_REG_HI_NUM ((int)sizeof(sk_bme280_humidity_comp_reg_hi))
#define I2C_BME280_DIGH_REG_LO_NUM ((int)sizeof(sk_bme280_humidity_comp_reg_lo))

// BME280 read pressure register "press"
static const uint8_t sk_bme280_press_reg[] = {
    0xF7, 0xF8, 0xF9,
};
#define I2C_BME280_PRESS_REG_NUM ((int)sizeof(sk_bme280_press_reg))

// BME280 read temperature register "temp"
static const uint8_t sk_bme280_temp_reg[] = {
    0xFA, 0xFB, 0xFC,
};
#define I2C_BME280_TEMP_REG_NUM ((int)sizeof(sk_bme280_temp_reg))

// BME280 read humidity register "hum"
static const uint8_t sk_bme280_hum_reg[] = {
    0xFD, 0xFE,
};
#define I2C_BME280_HUM_REG_NUM ((int)sizeof(sk_bme280_hum_reg))


namespace BME280
{

BME280::BME280( i2c_IO io )
    : m_IO( io )
{}

bool Initialize_i2c( i2c_port_t* port )
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        //.master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config( i2c_master_port, &conf );

    esp_err_t result = i2c_driver_install( i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0 );
    if( result != ESP_OK ){
        ESP_LOGE( sk_BME280_Tag, "Initialize_i2c() failed. reason=%s", esp_err_to_name(result) );
    }
    else {
        *port = i2c_master_port;
    }


    return result == ESP_OK;
}

/**
 * Read from sensor.
 * @param addr Register address.
 * @param out  Data to read.
 * @param size The number of bytes to read.
 * @returns true if success read data, otherwise false.
 */
bool i2c_IO::Read( uint8_t addr, uint8_t* out, size_t size )
{
    uint8_t slave_addr = this->slave << 1;
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if( cmd ){
        // Write register address
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, slave_addr | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, addr, true);

        // Read Registers
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, slave_addr | I2C_MASTER_READ, true);
        i2c_master_read(cmd, out, size, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);

        err = i2c_master_cmd_begin(this->i2cPort, cmd, CONFIG_BME280_TIMEOUT_TICKS);
        i2c_cmd_link_delete(cmd);

        return err == ESP_OK;
    }

    return false;
}

bool i2c_IO::Write( uint8_t addr, const uint8_t *in, size_t size )
{
    uint8_t slave_addr = this->slave << 1;
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if( cmd ){
        for( int i = 0; i < size; i++ ){
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, slave_addr | I2C_MASTER_WRITE, true);
            // Register
            i2c_master_write_byte(cmd, addr + i, true);
            // Data
            i2c_master_write_byte(cmd, in[i], true);
        }
        i2c_master_stop(cmd);

        err = i2c_master_cmd_begin(this->i2cPort, cmd, CONFIG_BME280_TIMEOUT_TICKS);
        i2c_cmd_link_delete(cmd);

        return err == ESP_OK;
    }

    return false;
}

bool BME280::Initialize()
{  
    uint8_t reg;
    uint8_t value;

    // set "config(0xF5)" register
    // t_sb[2:0]   = 0.5ms(000)
    // filter[2:0] = filter x16(100)
    // spi3w_en[0] = 3wire SPI(0)
    // value = |000|100|*|0|
    reg = 0xF5;
    value = 0x10;
    ESP_LOGE( sk_BME280_Tag, "%s set reg=0x%02X, value=0x%02X", __func__, reg, value );
    if( !m_IO.Write( reg, &value, 1 ) ){
        goto I2C_BME280_INIT_REG_BAILOUT;
    }

    // set "ctrl_meas(0xF4)" register
    // osrs_t[2:0]     = oversamplingx2(010)
    // osrs_p[2:0]     = oversamplingx16(101)
    // mode[1:0]       = normal mode(11)
    // value = |010|101|11|
    reg = 0xF4;
    value = 0x57;
    ESP_LOGE( sk_BME280_Tag, "%s set reg=0x%02X, value=0x%02X", __func__, reg, value );
    if( !m_IO.Write( reg, &value, 1 ) ){
        goto I2C_BME280_INIT_REG_BAILOUT;
    }

    // set "ctrl_hum(0xF2)" register
    // osrs_h[2:0]  = oversamplingx1(001)
    // value = |*****|001|
    reg = 0xF2;
    value = 0x01;
    ESP_LOGE( sk_BME280_Tag, "%s set reg=0x%02X, value=0x%02X", __func__, reg, value );
    if( !m_IO.Write( reg, &value, 1 ) ){
        goto I2C_BME280_INIT_REG_BAILOUT;
    }

    return true;

I2C_BME280_INIT_REG_BAILOUT:
    ESP_LOGE( sk_BME280_Tag, "%s command failed reg=0x%02X, value=0x%02X", __func__, reg, value );
    return false;
}

uint8_t BME280::ReadChipId()
{
    uint8_t chipid = 0;

    // read chipid
    m_IO.Read( 0xD0, &chipid, 1 );

    return chipid;
    //if( chipid != 0x60 ){
    //    ESP_LOGE( sk_BME280_Tag, "connected device is not bme280! chipid = 0x%02X\n", chipid );
    //}
}

bool BME280::ReadEnvMeasured( EnvData* envdata )
{
    uint8_t reg_press[I2C_BME280_PRESS_REG_NUM];
    uint8_t reg_temp[I2C_BME280_TEMP_REG_NUM];
    uint8_t reg_hum[I2C_BME280_HUM_REG_NUM];

    // read pressure data
    if( !m_IO.Read( sk_bme280_press_reg[0], reg_press, I2C_BME280_PRESS_REG_NUM ) ){
        return false;
    }
    // read temperature data
    if( !m_IO.Read( sk_bme280_temp_reg[0], reg_temp, I2C_BME280_TEMP_REG_NUM ) ){
        return false;
    }
    // read humidity data
    if( !m_IO.Read( sk_bme280_hum_reg[0], reg_hum, I2C_BME280_HUM_REG_NUM ) ){
        return false;
    }

    if( envdata ){
        envdata->pressure = (uint32_t)reg_press[0] << 16 | (uint32_t)reg_press[1] << 8 | (uint32_t)reg_press[2];
        envdata->pressure >>= 4;
        envdata->temperature = (uint32_t)reg_temp[0] << 16 | (uint32_t)reg_temp[1] << 8 | (uint32_t)reg_temp[2];
        envdata->temperature >>= 4;
        envdata->humidity = (uint32_t)reg_hum[0] << 8 | (uint32_t)reg_hum[1];
    }

    return true;
}

bool BME280::ReadCompensation( CompensationData* compdata )
{
    uint8_t reg_t[I2C_BME280_DIGT_REG_NUM];
    uint8_t reg_p[I2C_BME280_DIGP_REG_NUM];
    uint8_t reg_h[I2C_BME280_DIGH_REG_HI_NUM + I2C_BME280_DIGH_REG_LO_NUM];

    CompTemperature dig_t;
    CompPressure    dig_p;
    CompHumidity    dig_h;

    // read temperature compensation data
    if( !m_IO.Read( sk_bme280_temperature_comp_reg[0], reg_t, I2C_BME280_DIGT_REG_NUM ) ){
        return false;
    }
    // read pressure compensation data
    if( !m_IO.Read( sk_bme280_pressure_comp_reg[0], reg_p, I2C_BME280_DIGP_REG_NUM ) ){
        return false;
    }
    // read humidity compensation hi data
    if( !m_IO.Read( sk_bme280_humidity_comp_reg_hi[0], &reg_h[0], I2C_BME280_DIGH_REG_HI_NUM ) ){
        return false;
    }
    // read humidity compensation lo data
    if( !m_IO.Read( sk_bme280_humidity_comp_reg_lo[0], &reg_h[1], I2C_BME280_DIGH_REG_LO_NUM ) ){
        return false;
    }

    // ok. format compensation data.
    dig_t.t1 = (uint16_t)reg_t[0] | ((uint16_t)reg_t[1] << 8);
    dig_t.t2 = (int16_t)((uint16_t)reg_t[2] | ((uint16_t)reg_t[3] << 8));
    dig_t.t3 = (int16_t)((uint16_t)reg_t[4] | ((uint16_t)reg_t[5] << 8));

    dig_p.p1 = (uint16_t)reg_p[0] | ((uint16_t)reg_p[1] << 8);
    dig_p.p2 = (int16_t)((uint16_t)reg_p[2] | ((uint16_t)reg_p[3] << 8));
    dig_p.p3 = (int16_t)((uint16_t)reg_p[4] | ((uint16_t)reg_p[5] << 8));
    dig_p.p4 = (int16_t)((uint16_t)reg_p[6] | ((uint16_t)reg_p[7] << 8));
    dig_p.p5 = (int16_t)((uint16_t)reg_p[8] | ((uint16_t)reg_p[9] << 8));
    dig_p.p6 = (int16_t)((uint16_t)reg_p[10] | ((uint16_t)reg_p[11] << 8));
    dig_p.p7 = (int16_t)((uint16_t)reg_p[12] | ((uint16_t)reg_p[13] << 8));
    dig_p.p8 = (int16_t)((uint16_t)reg_p[14] | ((uint16_t)reg_p[15] << 8));
    dig_p.p9 = (int16_t)((uint16_t)reg_p[16] | ((uint16_t)reg_p[17] << 8));

    dig_h.h1 = reg_h[0];
    dig_h.h2 = (int16_t)((uint16_t)reg_h[1] | ((uint16_t)reg_h[2] << 8));
    dig_h.h3 = reg_h[3];
    dig_h.h4 = (int16_t)(((uint16_t)reg_h[4] << 4) | (uint16_t)(reg_h[5] & 0x0F));
    dig_h.h5 = (int16_t)(((uint16_t)reg_h[5] >> 4) | ((uint16_t)reg_h[6] << 8));
    dig_h.h6 = reg_h[7];

    if( compdata ){
        compdata->temperature = dig_t;
        compdata->pressure = dig_p;
        compdata->humidity = dig_h;
    }

    // succeeded
    return true;
}

float CalcTemperature( const CompTemperature& compt, int32_t temp )
{
    int32_t adc_T;
    int32_t var1, var2, T;
    int32_t t_fine_out;

    adc_T = temp;

    var1 = ((((adc_T >> 3) - ((int32_t)compt.t1 << 1))) * ((int32_t)compt.t2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)compt.t1)) * ((adc_T >> 4) - ((int32_t)compt.t1))) >> 12) * ((int32_t)compt.t3)) >> 14;
    
    t_fine_out = var1 + var2;
    T = ((t_fine_out * 5) + 128) >> 8;

    return T / 100.0f;
}

}    // BME280