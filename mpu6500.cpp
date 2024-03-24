#include "mpu6500.h"

#define Serial Serial1

// MPU-6500 Register Addresses
#define MPU6500_REG_WHO_AM_I          0x75

#define MPU6500_REG_ACCEL_XOUT_H      0x3B
#define MPU6500_REG_ACCEL_XOUT_L      0x3C
#define MPU6500_REG_ACCEL_YOUT_H      0x3D
#define MPU6500_REG_ACCEL_YOUT_L      0x3E
#define MPU6500_REG_ACCEL_ZOUT_H      0x3F
#define MPU6500_REG_ACCEL_ZOUT_L      0x40
#define MPU6500_REG_GYRO_XOUT_H       0x43
#define MPU6500_REG_GYRO_XOUT_L       0x44
#define MPU6500_REG_GYRO_YOUT_H       0x45
#define MPU6500_REG_GYRO_YOUT_L       0x46
#define MPU6500_REG_GYRO_ZOUT_H       0x47
#define MPU6500_REG_GYRO_ZOUT_L       0x48

#define MPU6500_REG_SMPLRT_DIV        0x19
#define MPU6500_REG_CONFIG            0x1A
#define MPU6500_REG_GYRO_CONFIG       0x1B
#define MPU6500_REG_ACCEL_CONFIG1     0x1C
#define MPU6500_REG_ACCEL_CONFIG2     0x1D
#define MPU6500_REG_FIFO_EN           0x23
#define MPU6500_REG_INT_PIN_CFG       0x37
#define MPU6500_REG_INT_ENABLE        0x38
#define MPU6500_REG_INT_STATUS        0x3A
#define MPU6500_REG_SIGNAL_PATH_RESET 0x68
#define MPU6500_REG_PWR_MGMT_1        0x6B

// When reading IMU data, we'll start at accel X
// which goes accel X,Y,Z,temperature, gyro X,Y,Z, (High, Low byte)
#define IMU_DATA_START_ADDR MPU6500_REG_ACCEL_XOUT_H

#define BITSHIFT_GYRO_DPS 3
#define BITSHIFT_ACCEL_G 3

#define GRAVITY 9.82

#define _low(pin) digitalWrite(pin, LOW);
#define _high(pin) digitalWrite(pin, HIGH);

// -- Typedefs -- //
typedef enum
{
    GYRO_DPS_250  = 0b00, // ±250dps
    GYRO_DPS_500  = 0b01, // ±500dps
    GYRO_DPS_1000 = 0b10, // ±1000dps
    GYRO_DPS_2000 = 0b11, // ±2000dps
} gyro_fs_t;

typedef enum
{
    ACCEL_FS_SEL_2G  = 0b00, // 2G
    ACCEL_FS_SEL_4G  = 0b01, // 4G
    ACCEL_FS_SEL_8G  = 0b10, // 8G
    ACCEL_FS_SEL_16G = 0b11, // 16G
} accel_fs_t;


// -- Instantiations -- //
static const int gyro_fs          = GYRO_DPS_2000;
static const int accel_fs         = ACCEL_FS_SEL_16G;
static const int gyro_raw_scaler  = GYRO_RAW_TO_DPS_2000;
static const int accel_raw_scaler = ACCEL_RAW_TO_G_16G;


MPU6500::MPU6500(HardwareSPI* spi, const uint8_t cs_pin, const uint32_t clock_speed)
: m_spi(spi),
  m_cs_pin(cs_pin),
  m_spisettings(clock_speed, MSBFIRST, SPI_MODE0)
{
    m_gyro_fs          = GYRO_DPS_2000;
    m_accel_fs         = ACCEL_FS_SEL_16G;
    m_gyro_raw_scaler  = GYRO_RAW_TO_DPS_2000;
    m_accel_raw_scaler = ACCEL_RAW_TO_G_16G;
}

bool MPU6500::init()
{
    Serial.printf("** Initializing IMU with CS pin: %d\n", m_cs_pin);

    pinMode(m_cs_pin, INPUT_PULLUP);
    _high(m_cs_pin);

    uint8_t id;
    read_reg(MPU6500_REG_WHO_AM_I, &id);
    Serial.printf("  Id: 0x%02x\n", id);

    // Datasheet says it should be 0x70, but on one module it's 0x78... Perhaps it's a clone?
    if (id != 0x70 && id != 0x78)
    {
        return false;
    }

    reset();

    uint8_t gyro_config, accel_config1, accel_config2, fifo_en, int_pin_cfg, int_enable, int_status, sample_div;
    // Read
    read_reg(MPU6500_REG_GYRO_CONFIG, &gyro_config);
    read_reg(MPU6500_REG_ACCEL_CONFIG1, &accel_config1);
    read_reg(MPU6500_REG_ACCEL_CONFIG2, &accel_config2);
    read_reg(MPU6500_REG_FIFO_EN, &fifo_en);
    read_reg(MPU6500_REG_INT_PIN_CFG, &int_pin_cfg);
    read_reg(MPU6500_REG_INT_ENABLE, &int_enable);
    read_reg(MPU6500_REG_INT_STATUS, &int_status);
    read_reg(MPU6500_REG_SMPLRT_DIV, &sample_div);

    Serial.printf("  Before config: Gyro config: %d, Accel config: %d, Accel config 2: %d, fifo_en: %d, int_pin_cfg: %d, int_enable: %d, int_status: %d, sample_div: %d\n",
                  gyro_config, accel_config1, accel_config2, fifo_en, int_pin_cfg, int_enable, int_status, sample_div);

    // Write
    write_reg(MPU6500_REG_GYRO_CONFIG, gyro_fs << BITSHIFT_GYRO_DPS);
    write_reg(MPU6500_REG_ACCEL_CONFIG1, accel_fs << BITSHIFT_ACCEL_G);
    write_reg(MPU6500_REG_ACCEL_CONFIG2, 0b100); // ACCEL_FCHOICE_B, 4 khz sampling freq
    write_reg(MPU6500_REG_INT_ENABLE, 1);
    write_reg(MPU6500_REG_SMPLRT_DIV, 2);

    // Read
    read_reg(MPU6500_REG_GYRO_CONFIG, &gyro_config);
    read_reg(MPU6500_REG_ACCEL_CONFIG1, &accel_config1);
    read_reg(MPU6500_REG_ACCEL_CONFIG2, &accel_config2);
    read_reg(MPU6500_REG_FIFO_EN, &fifo_en);
    read_reg(MPU6500_REG_INT_PIN_CFG, &int_pin_cfg);
    read_reg(MPU6500_REG_INT_ENABLE, &int_enable);
    read_reg(MPU6500_REG_INT_STATUS, &int_status);
    read_reg(MPU6500_REG_SMPLRT_DIV, &sample_div);

    Serial.printf("  After config: Gyro config: %d, Accel config: %d, Accel config 2: %d, fifo_en: %d, int_pin_cfg: %d, int_enable: %d, int_status: %d, sample_div: %d\n",
                  gyro_config, accel_config1, accel_config2, fifo_en, int_pin_cfg, int_enable, int_status, sample_div);

    Serial.printf("** Initialization for IMU with CS pin: %d done\n\n", m_cs_pin);

    return true;
}


void MPU6500::read_raw(imu_data_raw_t* raw)
{
    uint8_t buf[sizeof(imu_data_raw_t)];
    read_bytes(IMU_DATA_START_ADDR, buf, sizeof(imu_data_raw_t));
    raw->accel_x = (int16_t) (buf[0] << 8) | buf[1];
    raw->accel_y = (int16_t) (buf[2] << 8) | buf[3];
    raw->accel_z = (int16_t) (buf[4] << 8) | buf[5];
    raw->temp    = (int16_t) (buf[6] << 8) | buf[7];
    raw->gyro_x = (int16_t) (buf[8] << 8) | buf[9];
    raw->gyro_y = (int16_t) (buf[10] << 8) | buf[11];
    raw->gyro_z = (int16_t) (buf[12] << 8) | buf[13];
}

void MPU6500::scale_data(const imu_data_raw_t* raw, imu_data_scaled_t* scaled)
{
    scaled->accel_x = ((float) raw->accel_x / accel_raw_scaler) * GRAVITY;
    scaled->accel_y = ((float) raw->accel_y / accel_raw_scaler) * GRAVITY;
    scaled->accel_z = ((float) raw->accel_z / accel_raw_scaler) * GRAVITY;
    scaled->gyro_x  = (float) raw->gyro_x / gyro_raw_scaler;
    scaled->gyro_y  = (float) raw->gyro_y / gyro_raw_scaler;
    scaled->gyro_z  = (float) raw->gyro_z / gyro_raw_scaler;
    scaled->temp    = (float) raw->temp;
}

// -- Private -- //

void MPU6500::reset()
{
    write_reg(MPU6500_REG_PWR_MGMT_1, 0b10000000);
    delay(100);
    write_reg(MPU6500_REG_SIGNAL_PATH_RESET, 0b00000111);
    delay(100);
}

// -- SPI functions -- //
void MPU6500::write_reg(const uint8_t reg, uint8_t value)
{
    m_spi->beginTransaction(m_spisettings);
    _low(m_cs_pin);
    m_spi->transfer(reg);
    m_spi->transfer(value);
    _high(m_cs_pin);
    m_spi->endTransaction();
}

void MPU6500::read_bytes(const uint8_t reg, uint8_t* buf, size_t len)
{
    m_spi->beginTransaction(m_spisettings);
    _low(m_cs_pin);
    m_spi->transfer(reg | 0x80);
    for (int i = 0; i < len; i++)
    {
        buf[i] = m_spi->transfer(0x00);
    }
    _high(m_cs_pin);
    m_spi->endTransaction();
}

void MPU6500::read_reg(const uint8_t reg, uint8_t* value)
{
    m_spi->beginTransaction(m_spisettings);
    _low(m_cs_pin);
    m_spi->transfer(reg | 0x80);
    *value = m_spi->transfer(0x00);
    _high(m_cs_pin);
    m_spi->endTransaction();
}

void MPU6500::read_reg_16(const uint8_t reg, uint16_t* value)
{
    m_spi->beginTransaction(m_spisettings);
    _low(m_cs_pin);
    m_spi->transfer(reg | 0x80);
    *value = m_spi->transfer16(0x00);
    _high(m_cs_pin);
    m_spi->endTransaction();
}




