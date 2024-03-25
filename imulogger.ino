#include <SPI.h>
#include <ArduinoQueue.h>
#include <SD.h>

#include "mpu6500.h"

#define IMU_SAMPLING_RATE_HZ 4000
#define IMU_QUEUE_SIZE IMU_SAMPLING_RATE_HZ
#define Serial Serial1

#define PIN_CS_1 5
#define PIN_CS_2 6
#define PIN_SCK  2
#define PIN_MOSI 3
#define PIN_MISO 4
#define SPI_IMU          SPI

#define PIN_SD_CARD_SCK  10
#define PIN_SD_CARD_MOSI 11
#define PIN_SD_CARD_MISO 12
#define PIN_SD_CARD_CS   13
#define SPI_SD_CARD      SPI1

// -- Function definitions -- //
static void shell_run();
static void cmd_ls(String cmd);

// -- Instantiations -- //
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

volatile static uint64_t bytes_should_have_written = 0;
volatile static uint64_t bytes_have_written = 0;

static const int gyro_raw_scaler  = GYRO_RAW_TO_DPS_2000;
static const int accel_raw_scaler = ACCEL_RAW_TO_G_16G;

MPU6500 imu1(&SPI_IMU, PIN_CS_1);
MPU6500 imu2(&SPI_IMU, PIN_CS_2);


void setup() {
    rp2040.idleOtherCore();

    Serial.begin(115200);
    Serial.println("Booting up...");

    // IMU SPI
    SPI.setRX(PIN_MISO);
    SPI.setTX(PIN_MOSI);
    SPI.setSCK(PIN_SCK);
    SPI.begin();

    imu1.init();
    imu2.init();

    // SD Card SPI
    pinMode(PIN_SD_CARD_CS, INPUT_PULLUP);
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
        sprintf_P(buf, "Imus: 2, Type: MPU6500, Rate: %d, Scale_accel: %d, Scale_gyro: %d\n", IMU_SAMPLING_RATE_HZ, accel_raw_scaler, gyro_raw_scaler);
        current_log_file.truncate(0);
        current_log_file.write(buf, strlen(buf));
        current_log_file.flush();
    }

    rp2040.resumeOtherCore();
}


static uint32_t next_sample = 0;
static const uint32_t period_us = 250;
static uint32_t sampling_t0 = 0;
static uint32_t sample_fs_counter = 0;
static uint32_t sample_fs = 0;

void setup1()
{
    // We'll wait a bit to let main core initialize I/O
    delay(1000);
    next_sample = micros() + period_us;
    sampling_t0 = millis();
}

void loop1()
{
    static imu_data_raw_t raw_imu_1;
    static imu_data_raw_t raw_imu_2;

    next_sample += period_us;

    uint32_t t0 = micros();
    imu1.read_raw(&raw_imu_1);
    dt1 = micros() - t0;
    t0 = micros();
    imu2.read_raw(&raw_imu_2);
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

    static uint32_t last_sample_fs_update = 0;
    sample_fs_counter++;

    if ((millis() - last_sample_fs_update) > 1000)
    {
        sample_fs = sample_fs_counter;
        sample_fs_counter = 0;
        last_sample_fs_update = millis();
    }


    // Wait until next sample
    uint32_t time_to_sleep_us = next_sample - micros();
    if (time_to_sleep_us > 0)
    {
        delayMicroseconds(time_to_sleep_us);
    }
}

uint64_t avg_cnt = 0;
uint32_t avg_nbrs = 0;

void loop() {
    static uint32_t t0 = 0;
    static uint32_t last_debug_print = 0;

    if (!reader_queue->isEmpty())
    {
        while (!reader_queue->isEmpty())
        {
            // There's new data in the RX buffer, so we'll empty it and write to SD card
            last_imu_1_data_raw = reader_queue->dequeue();
            last_imu_2_data_raw = reader_queue->dequeue();
            imu1.scale_data(&last_imu_1_data_raw, &last_imu_1_data_scaled);
            imu2.scale_data(&last_imu_2_data_raw, &last_imu_2_data_scaled);
            bytes_should_have_written += 28;
            bytes_have_written += current_log_file.write((uint8_t*) &last_imu_1_data_raw, sizeof(imu_data_raw_t));
            bytes_have_written += current_log_file.write((uint8_t*) &last_imu_2_data_raw, sizeof(imu_data_raw_t));

        }
        current_log_file.flush();
    }

    if ((millis() - last_debug_print) >= 1000)
    {
        last_debug_print = millis();
        Serial.print("\n");
        Serial.printf("[1] Ax: %d,\tAy: %d,\tAz: %d,\tGx: %d,\tGy: %d,\tGz: %d,\tT: %d\n", last_imu_1_data_raw.accel_x, last_imu_1_data_raw.accel_y, last_imu_1_data_raw.accel_z, last_imu_1_data_raw.gyro_x, last_imu_1_data_raw.gyro_y, last_imu_1_data_raw.gyro_z, last_imu_1_data_raw.temp);
        Serial.printf("[1] Ax: %f,\tAy: %f,\tAz: %f,\tGx: %f,\tGy: %f,\tGz: %f,\tT: %f\n", last_imu_1_data_scaled.accel_x, last_imu_1_data_scaled.accel_y, last_imu_1_data_scaled.accel_z, last_imu_1_data_scaled.gyro_x, last_imu_1_data_scaled.gyro_y, last_imu_1_data_scaled.gyro_z, last_imu_1_data_scaled.temp);
        Serial.printf("[2] Ax: %d,\tAy: %d,\tAz: %d,\tGx: %d,\tGy: %d,\tGz: %d,\tT: %d\n", last_imu_2_data_raw.accel_x, last_imu_2_data_raw.accel_y, last_imu_2_data_raw.accel_z, last_imu_2_data_raw.gyro_x, last_imu_2_data_raw.gyro_y, last_imu_2_data_raw.gyro_z, last_imu_2_data_raw.temp);
        Serial.printf("[2] Ax: %f,\tAy: %f,\tAz: %f,\tGx: %f,\tGy: %f,\tGz: %f,\tT: %f\n", last_imu_2_data_scaled.accel_x, last_imu_2_data_scaled.accel_y, last_imu_2_data_scaled.accel_z, last_imu_2_data_scaled.gyro_x, last_imu_2_data_scaled.gyro_y, last_imu_2_data_scaled.gyro_z, last_imu_2_data_scaled.temp);
        Serial.printf("DT: (%d, %d) us, RE: %d, rate: %d hz (%llu total)\n", dt1, dt2, reader_not_empty, sample_fs, total_samples);
        Serial.printf("Written: %.2f / %.2f MB\n", (double) bytes_have_written / 1000000.0, (double) bytes_should_have_written / 1000000.0);
        //Serial.printf("Log file size: %.3f MB\n", (float) current_log_file.size() /f 1000000);
    }

}

// -- Misc functions -- //
static void shell_run()
{
    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');

        // If listing files, don't forget to close active log file!!
        //cmd_ls("asd");
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
