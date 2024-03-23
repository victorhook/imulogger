#include <SPI.h>

// MPU-6500 Register Addresses
#define WHO_AM_I        0x75
#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48
#define IMU_DATA_START_ADDR ACCEL_XOUT_H

#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG1 0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_FIFO_EN       0x23
#define REG_INT_PIN_CFG   0x37
#define REG_INT_ENABLE    0x38
#define REG_INT_STATUS    0x3A

#define PIN_CS   5
#define PIN_SCK  2
#define PIN_MOSI 3
#define PIN_MISO 4
#define PIN_IRQ  6

#define _low(pin) digitalWrite(pin, LOW);
#define _high(pin) digitalWrite(pin, HIGH);

#define Serial Serial1

SPISettings spisettings(3000000, MSBFIRST, SPI_MODE0);

static void read_reg_16(HardwareSPI* spi, SPISettings spisettings, const uint8_t cs_pin, const uint8_t reg, uint16_t* value);
static void read_reg(HardwareSPI* spi, SPISettings spisettings, const uint8_t cs_pin, const uint8_t reg, uint8_t* value);
static void read_bytes(HardwareSPI* spi, SPISettings spisettings, const uint8_t cs_pin, const uint8_t reg, uint8_t* buf, size_t len);

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

void setup() {
    Serial.begin(115200);

    pinMode(PIN_CS, INPUT_PULLUP);
    pinMode(PIN_IRQ, INPUT);

    SPI.setRX(PIN_MISO);
    SPI.setTX(PIN_MOSI);
    SPI.setSCK(PIN_SCK);
    SPI.setCS(PIN_CS);
    SPI.begin();

    Serial.println("Booting up...");

}


void read_data(imu_data_raw_t* raw)
{
    uint8_t buf[sizeof(imu_data_raw_t)];
    read_bytes(&SPI, spisettings, PIN_CS, IMU_DATA_START_ADDR, buf, sizeof(imu_data_raw_t));
    raw->accel_x = (buf[0] << 8) | buf[1];
    raw->accel_y = (buf[2] << 8) | buf[3];
    raw->accel_z = (buf[4] << 8) | buf[5];
    raw->temp    = (buf[6] << 8) | buf[7];
    raw->gyro_x = (buf[8] << 8) | buf[9];
    raw->gyro_y = (buf[10] << 8) | buf[11];
    raw->gyro_z = (buf[12] << 8) | buf[13];
}

void scale_data(const imu_data_raw_t* raw, imu_data_scaled_t* scaled)
{
    scaled->accel_x = (float) raw->accel_x;
    scaled->accel_y = (float) raw->accel_y;
    scaled->accel_z = (float) raw->accel_z;
    scaled->temp = (float) raw->temp;
    scaled->gyro_x = (float) raw->gyro_x;
    scaled->gyro_y = (float) raw->gyro_y;
    scaled->gyro_z = (float) raw->gyro_z;
}

imu_data_raw_t last_imu_data_raw;
imu_data_scaled_t last_imu_data_scaled;

typedef enum
{
    GYRO_DPS_250  = 0b00, // = ±250dps
    GYRO_DPS_500  = 0b01, //= ±500dps
    GYRO_DPS_1000 = 0b10, // = ±1000dps
    GYRO_DPS_2000 = 0b11, // = ±2000dps
} gyro_fs_t;

typedef enum
{
    ACCEL_FS_SEL_2G  = 0b00, // 2G
    ACCEL_FS_SEL_4G  = 0b01, // 4G
    ACCEL_FS_SEL_8G  = 0b10, // 8G
    ACCEL_FS_SEL_16G = 0b11, // 16G
} accel_fs_t;

#define BITSHIFT_GYRO_DPS 3
#define BITSHIFT_ACCEL_G 3

static void write_reg(HardwareSPI* spi, SPISettings spisettings, const uint8_t cs_pin, const uint8_t reg, uint8_t value)
{
    spi->beginTransaction(spisettings);
    _low(cs_pin);
    spi->transfer(reg);
    spi->transfer(value);
    _high(cs_pin);
    spi->endTransaction();
}

void loop() {
    /*
    uint8_t gyro_config, accel_config1, accel_config2, fifo_en, int_pin_cfg, int_enable, int_status, sample_div;
    read_reg(&SPI, spisettings, PIN_CS, REG_GYRO_CONFIG, &gyro_config);
    read_reg(&SPI, spisettings, PIN_CS, REG_ACCEL_CONFIG1, &accel_config1);
    read_reg(&SPI, spisettings, PIN_CS, REG_ACCEL_CONFIG2, &accel_config2);
    read_reg(&SPI, spisettings, PIN_CS, REG_FIFO_EN, &fifo_en);
    read_reg(&SPI, spisettings, PIN_CS, REG_INT_PIN_CFG, &int_pin_cfg);
    read_reg(&SPI, spisettings, PIN_CS, REG_INT_ENABLE, &int_enable);
    read_reg(&SPI, spisettings, PIN_CS, REG_INT_STATUS, &int_status);
    read_reg(&SPI, spisettings, PIN_CS, REG_SMPLRT_DIV, &sample_div);

    Serial.printf("Gyro config: %d, Accel config: %d, Accel config 2: %d, fifo_en: %d, int_pin_cfg: %d, int_enable: %d, int_status: %d, sample_div: %d\n",
                  gyro_config, accel_config1, accel_config2, fifo_en, int_pin_cfg, int_enable, int_status, sample_div);

    write_reg(&SPI, spisettings, PIN_CS, REG_GYRO_CONFIG, GYRO_DPS_2000 << BITSHIFT_GYRO_DPS);
    write_reg(&SPI, spisettings, PIN_CS, REG_ACCEL_CONFIG1, ACCEL_FS_SEL_16G << BITSHIFT_ACCEL_G);
    write_reg(&SPI, spisettings, PIN_CS, REG_INT_ENABLE, 1);
    delay(2000);
    */

    int16_t buf[7];

    uint32_t t0 = micros();
    read_data(&last_imu_data_raw);
    uint32_t dt = micros() - t0;

    scale_data(&last_imu_data_raw, &last_imu_data_scaled);

    Serial.printf("%d, %d, %d, %d, %d, %d, %d\n", last_imu_data_raw.accel_x, last_imu_data_raw.accel_y, last_imu_data_raw.accel_z, last_imu_data_raw.gyro_x, last_imu_data_raw.gyro_y, last_imu_data_raw.gyro_z, last_imu_data_raw.temp);
    Serial.printf("%f, %f, %f, %f, %f, %f, %f\n", last_imu_data_scaled.accel_x, last_imu_data_scaled.accel_y, last_imu_data_scaled.accel_z, last_imu_data_scaled.gyro_x, last_imu_data_scaled.gyro_y, last_imu_data_scaled.gyro_z, last_imu_data_scaled.temp);
    Serial.printf("DT: %d us\n", dt);
    Serial.print("\n");
    delay(2000);
}


static void read_bytes(HardwareSPI* spi, SPISettings spisettings, const uint8_t cs_pin, const uint8_t reg, uint8_t* buf, size_t len)
{
    spi->beginTransaction(spisettings);
    _low(cs_pin);
    spi->transfer(reg | 0x80);
    for (int i = 0; i < len; i++)
    {
        buf[i] = spi->transfer(0x00);
    }
    _high(cs_pin);
    spi->endTransaction();
}


static void read_reg(HardwareSPI* spi, SPISettings spisettings, const uint8_t cs_pin, const uint8_t reg, uint8_t* value)
{
    spi->beginTransaction(spisettings);
    _low(cs_pin);
    spi->transfer(reg | 0x80);
    *value = spi->transfer(0x00);
    _high(cs_pin);
    spi->endTransaction();
}

static void read_reg_16(HardwareSPI* spi, SPISettings spisettings, const uint8_t cs_pin, const uint8_t reg, uint16_t* value)
{
    spi->beginTransaction(spisettings);
    _low(cs_pin);
    spi->transfer(reg | 0x80);
    *value = spi->transfer16(0x00);
    _high(cs_pin);
    spi->endTransaction();
}




