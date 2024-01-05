#ifndef I2C_BME280_H_INCLUDED
#define I2C_BME280_H_INCLUDED

#include <cstdint>

// esp-idf header
#include "driver/i2c.h"

namespace BME280
{

static constexpr uint8_t sk_SlaveAddr = 0x76;
static constexpr uint8_t sk_ChipId = 0x60;

bool Initialize_i2c( i2c_port_t* port );

struct CompTemperature
{
    uint16_t t1;
    int16_t t2;
    int16_t t3;
};

struct CompPressure
{
    uint16_t p1;
    int16_t p2;
    int16_t p3;
    int16_t p4;
    int16_t p5;
    int16_t p6;
    int16_t p7;
    int16_t p8;
    int16_t p9;
};

struct CompHumidity
{
    uint8_t h1;
    int16_t h2;
    uint8_t h3;
    int16_t h4;
    int16_t h5;
    int8_t  h6;
};

struct CompensationData
{
    CompTemperature temperature;
    CompPressure pressure;
    CompHumidity humidity;
};

struct EnvData
{
    int32_t pressure;
    int32_t temperature;
    int32_t humidity;
};

struct i2c_IO
{
    i2c_port_t i2cPort;
    uint8_t slave;

    bool Read(uint8_t addr, uint8_t* out, size_t size);
    bool Write(uint8_t addr, const uint8_t* in, size_t size);
};

class BME280
{
public:

    BME280( i2c_IO io );
    ~BME280() = default;

    bool Initialize();
    uint8_t ReadChipId();
    bool ReadEnvMeasured( EnvData* envdata );
    bool ReadCompensation( CompensationData* compdata );

private:

    i2c_IO m_IO;
};

float CalcTemperature( const CompTemperature& compt, int32_t temp );

}    // namespace BME280

#endif      // I2C_BME280_H_INCLUDED