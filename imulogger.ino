#include <SPI.h>
#include <ArduinoQueue.h>
#include <SD.h>


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
#define REG_SIGNAL_PATH_RESET 0x68
#define REG_PWR_MGMT_1    0x6B

#define ACCEL_RAW_TO_G_2G  16384
#define ACCEL_RAW_TO_G_4G  8192
#define ACCEL_RAW_TO_G_8G  4096
#define ACCEL_RAW_TO_G_16G 2048

#define GYRO_RAW_TO_DPS_250  131
#define GYRO_RAW_TO_DPS_500  65.5
#define GYRO_RAW_TO_DPS_1000 32.8
#define GYRO_RAW_TO_DPS_2000 16.4

#define BITSHIFT_GYRO_DPS 3
#define BITSHIFT_ACCEL_G 3

#define PIN_CS_1 5
#define PIN_CS_2 6
#define PIN_SCK  2
#define PIN_MOSI 3
#define PIN_MISO 4

#define PIN_SD_CARD_SCK  10
#define PIN_SD_CARD_MOSI 11
#define PIN_SD_CARD_MISO 12
#define PIN_SD_CARD_CS   13
#define SPI_SD_CARD      SPI1

#define _low(pin) digitalWrite(pin, LOW);
#define _high(pin) digitalWrite(pin, HIGH);

#define IMU_SAMPLING_RATE_HZ 4000

#define Serial Serial1

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

#define GRAVITY 9.82


// -- Function definitions -- //
static void read_reg_16(HardwareSPI* spi, SPISettings spisettings, const uint8_t cs_pin, const uint8_t reg, uint16_t* value);
static void read_reg(HardwareSPI* spi, SPISettings spisettings, const uint8_t cs_pin, const uint8_t reg, uint8_t* value);
static void read_bytes(HardwareSPI* spi, SPISettings spisettings, const uint8_t cs_pin, const uint8_t reg, uint8_t* buf, size_t len);
static void write_reg(HardwareSPI* spi, SPISettings spisettings, const uint8_t cs_pin, const uint8_t reg, uint8_t value);

static bool init_imu(const uint8_t pin_cs);
static void imu_reset(const uint8_t pin_cs);
static void read_data(imu_data_raw_t* raw);
static void scale_data(const imu_data_raw_t* raw, imu_data_scaled_t* scaled);
static void imu_irq();

static void shell_run();
static void cmd_ls(String cmd);

// -- Instantiations -- //
static const int gyro_fs          = GYRO_DPS_2000;
static const int accel_fs         = ACCEL_FS_SEL_16G;
static const int gyro_raw_scaler  = GYRO_RAW_TO_DPS_2000;
static const int accel_raw_scaler = ACCEL_RAW_TO_G_16G;

#define IMU_QUEUE_SIZE 3000
static ArduinoQueue<imu_data_raw_t> queue1(IMU_QUEUE_SIZE);
static ArduinoQueue<imu_data_raw_t> queue2(IMU_QUEUE_SIZE);

static int active_queue_writer = 1;
static ArduinoQueue<imu_data_raw_t>* reader_queue = &queue2;
static uint64_t total_samples = 0;
static uint32_t reader_not_empty = 0;

File current_log_file;

imu_data_raw_t last_imu_1_data_raw;
imu_data_raw_t last_imu_2_data_raw;
imu_data_scaled_t last_imu_1_data_scaled;
imu_data_scaled_t last_imu_2_data_scaled;
uint32_t dt1 = 0;
uint32_t dt2 = 0;
SPISettings spisettings(3000000, MSBFIRST, SPI_MODE0);


void setup() {
    rp2040.idleOtherCore();

    Serial.begin(115200);
    Serial.println("Booting up...");

    pinMode(PIN_CS_1, INPUT_PULLUP);
    pinMode(PIN_CS_2, INPUT_PULLUP);
    pinMode(PIN_SD_CARD_CS, INPUT_PULLUP);

    SPI.setRX(PIN_MISO);
    SPI.setTX(PIN_MOSI);
    SPI.setSCK(PIN_SCK);
    SPI.begin();

    init_imu(PIN_CS_1);
    init_imu(PIN_CS_2);

    SPI_SD_CARD.setRX(PIN_SD_CARD_MISO);
    SPI_SD_CARD.setTX(PIN_SD_CARD_MOSI);
    SPI_SD_CARD.setSCK(PIN_SD_CARD_SCK);
    SPI_SD_CARD.begin();

    if (!SD.begin(PIN_SD_CARD_CS, SPI_SD_CARD))
    {
        Serial.println("Failed to initialize SD card!");
    }
    else
    {
        //File root = SD.open("/log.txt", FILE_WRITE);
        //root.write("Hello world!\n");
        //root.close();
        current_log_file = SD.open("/log.bin", FILE_WRITE);
        char buf[128];
        sprintf_P(buf, "Imus: 2, Type: MPU6500, Rate: %d, Scale_accel: %d, Scale_gyro: %d\n", IMU_SAMPLING_RATE_HZ, ACCEL_RAW_TO_G_16G, GYRO_RAW_TO_DPS_2000);
        current_log_file.write(buf, strlen(buf));
        current_log_file.flush();
    }

    rp2040.resumeOtherCore();
}


static uint32_t next_sample = 0;
static const uint32_t period_us = 250;

void setup1()
{
    // We'll wait a bit to let main core initialize I/O
    delay(100);
    next_sample = micros() + period_us;
}

void loop1()
{
    static imu_data_raw_t raw_imu_1;
    static imu_data_raw_t raw_imu_2;

    uint32_t t0 = micros();
    read_data(&raw_imu_1, PIN_CS_1);
    dt1 = micros() - t0;
    t0 = micros();
    read_data(&raw_imu_2, PIN_CS_2);
    dt2 = micros() - t0;

    total_samples++;

    ArduinoQueue<imu_data_raw_t>* queue = (active_queue_writer == 1) ? &queue1 : &queue2;
    queue->enqueue(raw_imu_1);
    queue->enqueue(raw_imu_2);

    if (queue->isFull())
    {
      rp2040.idleOtherCore();

      if (!reader_queue->isEmpty())
      {
        reader_not_empty++;
      }

      // Swap buffers
      active_queue_writer = (active_queue_writer == 1) ? 2 : 1;
      reader_queue = (active_queue_writer == 1) ? &queue2 : &queue1;


      rp2040.resumeOtherCore();
    }

    // Wait until next sample
    uint32_t time_to_sleep_us = next_sample - micros();
    if (time_to_sleep_us > 0)
    {
        delayMicroseconds(time_to_sleep_us);
    }
    
    next_sample = micros() + period_us;
}


void loop() {
    static uint32_t t0 = 0;
    static uint32_t last_debug_print = 0;

    if (!reader_queue->isEmpty())
    {
      last_imu_1_data_raw = reader_queue->dequeue();
      last_imu_2_data_raw = reader_queue->dequeue();
      scale_data(&last_imu_1_data_raw, &last_imu_1_data_scaled);
      scale_data(&last_imu_2_data_raw, &last_imu_2_data_scaled);
      current_log_file.write((uint8_t*) &last_imu_1_data_raw, sizeof(imu_data_raw_t));
      current_log_file.write((uint8_t*) &last_imu_2_data_raw, sizeof(imu_data_raw_t));
    }

    if ((millis() - last_debug_print) >= 1000)
    {
        current_log_file.flush();
        static uint64_t last_total_samples = 0;
        uint32_t d_samples = total_samples - last_total_samples;
        last_total_samples = total_samples;
        Serial.printf("[1] Ax: %d,\tAy: %d,\tAz: %d,\tGx: %d,\tGy: %d,\tGz: %d,\tT: %d\n", last_imu_1_data_raw.accel_x, last_imu_1_data_raw.accel_y, last_imu_1_data_raw.accel_z, last_imu_1_data_raw.gyro_x, last_imu_1_data_raw.gyro_y, last_imu_1_data_raw.gyro_z, last_imu_1_data_raw.temp);
        Serial.printf("[1] Ax: %f,\tAy: %f,\tAz: %f,\tGx: %f,\tGy: %f,\tGz: %f,\tT: %f\n", last_imu_1_data_scaled.accel_x, last_imu_1_data_scaled.accel_y, last_imu_1_data_scaled.accel_z, last_imu_1_data_scaled.gyro_x, last_imu_1_data_scaled.gyro_y, last_imu_1_data_scaled.gyro_z, last_imu_1_data_scaled.temp);
        Serial.printf("[2] Ax: %d,\tAy: %d,\tAz: %d,\tGx: %d,\tGy: %d,\tGz: %d,\tT: %d\n", last_imu_2_data_raw.accel_x, last_imu_2_data_raw.accel_y, last_imu_2_data_raw.accel_z, last_imu_2_data_raw.gyro_x, last_imu_2_data_raw.gyro_y, last_imu_2_data_raw.gyro_z, last_imu_2_data_raw.temp);
        Serial.printf("[2] Ax: %f,\tAy: %f,\tAz: %f,\tGx: %f,\tGy: %f,\tGz: %f,\tT: %f\n", last_imu_2_data_scaled.accel_x, last_imu_2_data_scaled.accel_y, last_imu_2_data_scaled.accel_z, last_imu_2_data_scaled.gyro_x, last_imu_2_data_scaled.gyro_y, last_imu_2_data_scaled.gyro_z, last_imu_2_data_scaled.temp);
        Serial.printf("DT: (%d, %d) us, RE: %d, rate: %d fs (%llu total)\n", dt1, dt2, reader_not_empty, d_samples, total_samples);
        Serial.print("\n");
        cmd_ls("asd");
        last_debug_print = millis();
    }

}

// -- Misc functions -- //
static void shell_run()
{
    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');

        if (cmd.startsWith("ls"))
        {
            cmd_ls(cmd);
        }

    }
}

static void cmd_ls(String cmd)
{
    File root = SD.open("/");

    if (!root)
    {
        Serial.println("Failed to open root!");
        return;
    }

    while (true) {
        File entry = root.openNextFile();
        if (!entry)
        {
            // No more files
            break;
        }
        Serial.print("  ");
        Serial.println(entry.name());
        entry.close();
    }
    root.close();

}

// -- IMU functions -- //
static bool init_imu(const uint8_t pin_cs)
{
    Serial.printf("** Initializing IMU with CS pin: %d\n", pin_cs);

    uint8_t id;
    read_reg(&SPI, spisettings, pin_cs, WHO_AM_I, &id);
    Serial.printf("  Id: 0x%02x\n", id);

    // Datasheet says it should be 0x70, but on one module it's 0x78... Perhaps it's a clone?
    if (id != 0x70 && id != 0x78)
    {
        return false;
    }

    imu_reset(pin_cs);

    uint8_t gyro_config, accel_config1, accel_config2, fifo_en, int_pin_cfg, int_enable, int_status, sample_div;
    // Read
    read_reg(&SPI, spisettings, pin_cs, REG_GYRO_CONFIG, &gyro_config);
    read_reg(&SPI, spisettings, pin_cs, REG_ACCEL_CONFIG1, &accel_config1);
    read_reg(&SPI, spisettings, pin_cs, REG_ACCEL_CONFIG2, &accel_config2);
    read_reg(&SPI, spisettings, pin_cs, REG_FIFO_EN, &fifo_en);
    read_reg(&SPI, spisettings, pin_cs, REG_INT_PIN_CFG, &int_pin_cfg);
    read_reg(&SPI, spisettings, pin_cs, REG_INT_ENABLE, &int_enable);
    read_reg(&SPI, spisettings, pin_cs, REG_INT_STATUS, &int_status);
    read_reg(&SPI, spisettings, pin_cs, REG_SMPLRT_DIV, &sample_div);

    Serial.printf("  Before config: Gyro config: %d, Accel config: %d, Accel config 2: %d, fifo_en: %d, int_pin_cfg: %d, int_enable: %d, int_status: %d, sample_div: %d\n",
                  gyro_config, accel_config1, accel_config2, fifo_en, int_pin_cfg, int_enable, int_status, sample_div);

    // Write
    write_reg(&SPI, spisettings, pin_cs, REG_GYRO_CONFIG, gyro_fs << BITSHIFT_GYRO_DPS);
    write_reg(&SPI, spisettings, pin_cs, REG_ACCEL_CONFIG1, accel_fs << BITSHIFT_ACCEL_G);
    write_reg(&SPI, spisettings, pin_cs, REG_ACCEL_CONFIG2, 0b100); // ACCEL_FCHOICE_B, 4 khz sampling freq
    write_reg(&SPI, spisettings, pin_cs, REG_INT_ENABLE, 1);
    write_reg(&SPI, spisettings, pin_cs, REG_SMPLRT_DIV, 2);

    // Read
    read_reg(&SPI, spisettings, pin_cs, REG_GYRO_CONFIG, &gyro_config);
    read_reg(&SPI, spisettings, pin_cs, REG_ACCEL_CONFIG1, &accel_config1);
    read_reg(&SPI, spisettings, pin_cs, REG_ACCEL_CONFIG2, &accel_config2);
    read_reg(&SPI, spisettings, pin_cs, REG_FIFO_EN, &fifo_en);
    read_reg(&SPI, spisettings, pin_cs, REG_INT_PIN_CFG, &int_pin_cfg);
    read_reg(&SPI, spisettings, pin_cs, REG_INT_ENABLE, &int_enable);
    read_reg(&SPI, spisettings, pin_cs, REG_INT_STATUS, &int_status);
    read_reg(&SPI, spisettings, pin_cs, REG_SMPLRT_DIV, &sample_div);

    Serial.printf("  After config: Gyro config: %d, Accel config: %d, Accel config 2: %d, fifo_en: %d, int_pin_cfg: %d, int_enable: %d, int_status: %d, sample_div: %d\n",
                  gyro_config, accel_config1, accel_config2, fifo_en, int_pin_cfg, int_enable, int_status, sample_div);

    Serial.printf("** Initialization for IMU with CS pin: %d done\n\n", pin_cs);

    return true;
}

static void imu_reset(const uint8_t pin_cs)
{
    write_reg(&SPI, spisettings, pin_cs, REG_PWR_MGMT_1, 0b10000000);
    delay(100);
    write_reg(&SPI, spisettings, pin_cs, REG_SIGNAL_PATH_RESET, 0b00000111);
    delay(100);
}

static void read_data(imu_data_raw_t* raw, const uint8_t pin_cs)
{
    uint8_t buf[sizeof(imu_data_raw_t)];
    read_bytes(&SPI, spisettings, pin_cs, IMU_DATA_START_ADDR, buf, sizeof(imu_data_raw_t));
    raw->accel_x = (int16_t) (buf[0] << 8) | buf[1];
    raw->accel_y = (int16_t) (buf[2] << 8) | buf[3];
    raw->accel_z = (int16_t) (buf[4] << 8) | buf[5];
    raw->temp    = (int16_t) (buf[6] << 8) | buf[7];
    raw->gyro_x = (int16_t) (buf[8] << 8) | buf[9];
    raw->gyro_y = (int16_t) (buf[10] << 8) | buf[11];
    raw->gyro_z = (int16_t) (buf[12] << 8) | buf[13];
}

static void scale_data(const imu_data_raw_t* raw, imu_data_scaled_t* scaled)
{
    scaled->accel_x = ((float) raw->accel_x / accel_raw_scaler) * GRAVITY;
    scaled->accel_y = ((float) raw->accel_y / accel_raw_scaler) * GRAVITY;
    scaled->accel_z = ((float) raw->accel_z / accel_raw_scaler) * GRAVITY;
    scaled->gyro_x  = (float) raw->gyro_x / gyro_raw_scaler;
    scaled->gyro_y  = (float) raw->gyro_y / gyro_raw_scaler;
    scaled->gyro_z  = (float) raw->gyro_z / gyro_raw_scaler;
    scaled->temp    = (float) raw->temp;
}


// -- SPI functions -- //
static void write_reg(HardwareSPI* spi, SPISettings spisettings, const uint8_t cs_pin, const uint8_t reg, uint8_t value)
{
    spi->beginTransaction(spisettings);
    _low(cs_pin);
    spi->transfer(reg);
    spi->transfer(value);
    _high(cs_pin);
    spi->endTransaction();
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




