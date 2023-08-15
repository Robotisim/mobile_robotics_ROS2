#include <pigpiod_if2.h>
#include <iostream>

const int MPU9250_ADDRESS = 0x68; // MPU9250 address
const int AK8963_ADDRESS = 0x0C; // AK8963 address
const int I2Cbus = 1; // RPi typical I2C bus

int pi = -1;
int MPU9250_HANDLE = -1;

using namespace std;

void setup_mpu9250()
{
    // Wake up MPU9250
    i2c_write_byte_data(pi, MPU9250_HANDLE, 0x6B, 0x00);
    time_sleep(0.1);

    // Enable bypass mode to access AK8963 directly
    i2c_write_byte_data(pi, MPU9250_HANDLE, 0x37, 0x02);
    time_sleep(0.1);

    // Open AK8963
    int AK8963_HANDLE = i2c_open(pi, I2Cbus, AK8963_ADDRESS, 0);
    if (AK8963_HANDLE < 0)
    {
        cout << "Unable to open I2C comms with AK8963" << endl;
        return;
    }

    // Reset AK8963
    i2c_write_byte_data(pi, AK8963_HANDLE, 0x0B, 0x01);
    time_sleep(0.1);

    // Set AK8963 to continuous measurement mode
    i2c_write_byte_data(pi, AK8963_HANDLE, 0x0A, 0x16); // 16-bit output, continuous mode 2
    time_sleep(0.1);

    i2c_close(pi, AK8963_HANDLE);
}

void get_data()
{
    // Read accelerometer and gyroscope data
    int16_t ax = (int16_t)(i2c_read_byte_data(pi, MPU9250_HANDLE, 0x3B) << 8 | i2c_read_byte_data(pi, MPU9250_HANDLE, 0x3C));
    int16_t ay = (int16_t)(i2c_read_byte_data(pi, MPU9250_HANDLE, 0x3D) << 8 | i2c_read_byte_data(pi, MPU9250_HANDLE, 0x3E));
    int16_t az = (int16_t)(i2c_read_byte_data(pi, MPU9250_HANDLE, 0x3F) << 8 | i2c_read_byte_data(pi, MPU9250_HANDLE, 0x40));
    int16_t gx = (int16_t)(i2c_read_byte_data(pi, MPU9250_HANDLE, 0x43) << 8 | i2c_read_byte_data(pi, MPU9250_HANDLE, 0x44));
    int16_t gy = (int16_t)(i2c_read_byte_data(pi, MPU9250_HANDLE, 0x45) << 8 | i2c_read_byte_data(pi, MPU9250_HANDLE, 0x46));
    int16_t gz = (int16_t)(i2c_read_byte_data(pi, MPU9250_HANDLE, 0x47) << 8 | i2c_read_byte_data(pi, MPU9250_HANDLE, 0x48));

    cout << "Accelerometer - X: " << ax << ", Y: " << ay << ", Z: " << az << endl;
    cout << "Gyroscope - X: " << gx << ", Y: " << gy << ", Z: " << gz << endl;

    // Open AK8963
    int AK8963_HANDLE = i2c_open(pi, I2Cbus, AK8963_ADDRESS, 0);

    // Read magnetometer data
    int16_t mx = (int16_t)(i2c_read_byte_data(pi, AK8963_HANDLE, 0x03) << 8 | i2c_read_byte_data(pi, AK8963_HANDLE, 0x04));
    int16_t my = (int16_t)(i2c_read_byte_data(pi, AK8963_HANDLE, 0x05) << 8 | i2c_read_byte_data(pi, AK8963_HANDLE, 0x06));
    int16_t mz = (int16_t)(i2c_read_byte_data(pi, AK8963_HANDLE, 0x07) << 8 | i2c_read_byte_data(pi, AK8963_HANDLE, 0x08));

    cout << "Magnetometer - X: " << mx << ", Y: " << my << ", Z: " << mz << endl;

    i2c_close(pi, AK8963_HANDLE);
}

int main()
{
    char *addrStr = NULL;
    char *portStr = NULL;
    pi = pigpio_start(addrStr, portStr);
    if (pi >= 0)
    {
        cout << "daemon interface started ok at " << pi << endl;
    }
    else
    {
        cout << "Failed to connect to PIGPIO Daemon - is it running?" << endl;
        return -1;
    }

    MPU9250_HANDLE = i2c_open(pi, I2Cbus, MPU9250_ADDRESS, 0);

    setup_mpu9250();

    while (true)
    {
        get_data();
        time_sleep(1); // Sleep for 1 second
    }

    i2c_close(pi, MPU9250_HANDLE);
    pigpio_stop(pi);
    return 0;
}
