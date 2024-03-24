#ifndef MPU_6500_H
#define MPU_6500_H

#include <Arduino.h>
#include <SPI.h>

#define ACCEL_RAW_TO_G_2G  16384
#define ACCEL_RAW_TO_G_4G  8192
#define ACCEL_RAW_TO_G_8G  4096
#define ACCEL_RAW_TO_G_16G 2048

#define GYRO_RAW_TO_DPS_250  131
#define GYRO_RAW_TO_DPS_500  65.5
#define GYRO_RAW_TO_DPS_1000 32.8
#define GYRO_RAW_TO_DPS_2000 16.4

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
}__attribute__((packed)) imu_data_raw_t;

typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;
    float temp;
    float gyro_x;
    float gyro_y;
    float gyro_z;
}__attribute__((packed)) imu_data_scaled_t;

class MPU6500
{
    public:
        MPU6500(HardwareSPI* spi, const uint8_t cs_pin, const uint32_t clock_speed = 3000000);
        bool init();
        void read_raw(imu_data_raw_t* raw);
        void scale_data(const imu_data_raw_t* raw, imu_data_scaled_t* scaled);

    private:
        HardwareSPI*  m_spi;
        const uint8_t m_cs_pin;
        SPISettings   m_spisettings;
        int           m_gyro_fs;
        int           m_accel_fs;
        int           m_gyro_raw_scaler;
        int           m_accel_raw_scaler;

        void read_reg_16(const uint8_t reg, uint16_t* value);
        void read_reg(const uint8_t reg, uint8_t* value);
        void read_bytes(const uint8_t reg, uint8_t* buf, size_t len);
        void write_reg(const uint8_t reg, uint8_t value);
        void reset();
};


#endif /* MPU_6500_H */
