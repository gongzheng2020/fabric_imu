#ifndef AHRS_H
#define AHRS_H

#include "Arduino.h"

// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
// -259.91/269.59  -278.09/256.12  -388.37/157.56
#define ACCEL_X_MIN ((float)-1958.00)
#define ACCEL_X_MAX ((float)2181.00)
#define ACCEL_Y_MIN ((float)-2142.00)
#define ACCEL_Y_MAX ((float)2142.00)
#define ACCEL_Z_MIN ((float)-2130.00)
#define ACCEL_Z_MAX ((float)2224.00)
// #define DEBUG_DRIFT_CORRECTION

// Magnetometer (standard calibration)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
// -682.27/263.36  -635.00/430.45  -533.18/493.36
#define MAGN_X_MIN ((float)-2117.00)
#define MAGN_X_MAX ((float)9797.00)
#define MAGN_Y_MIN ((float)-6515.00)
#define MAGN_Y_MAX ((float)5412.00)
#define MAGN_Z_MIN ((float)-6167.00)
#define MAGN_Z_MAX ((float)5960.00)

// Magnetometer (extended calibration)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
// #define CALIBRATION__MAGN_USE_EXTENDED
// const float magn_ellipsoid_center[3] = {0, 0, 0};
// const float magn_ellipsoid_transform[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
// 17.07/16.51  100.62/99.85  -26.05/-25.94
#define GYRO_AVERAGE_OFFSET_X ((float)17.33)
#define GYRO_AVERAGE_OFFSET_Y ((float)36.25)
#define GYRO_AVERAGE_OFFSET_Z ((float)-2.64)

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))
#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))
// Gain for gyroscope (MPU6050)
#define GYRO_GAIN 0.06098                         // Same gain on all axes(LSB_2000)
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second
// Magnetometer gain
#define MAGN_SCALED_GAUSS(x) (x * 0.000061035f) // Calculate the scaled magnetometer readings in Gauss
// DCM parameters
#define Kp_ROLLPITCH 0.005f
#define Ki_ROLLPITCH 0.00000005f
#define Kp_YAW 2.5f
#define Ki_YAW 0.000005f
// Stuff
#define GRAVITY 2048.0f              // "8G reference" used for DCM filter and accelerometer calibration(LSB_2G)
#define GRAVITY_ACCELERATION 9.80665f // Standard gravity acceleration in m/s^2
#define TO_RAD(x) (x * 0.01745329252) // *pi/180
#define TO_DEG(x) (x * 57.2957795131) // *180/pi

// 联合体定义，用于 float 和字节数组之间的转换
union FloatToBytes {
    float f;
    uint8_t b[4];
};

enum class SensorType
{
    Euler,
    Accel,
    Magnetom,
    Gyro,
    Invalid
};
enum class Axis
{
    X,
    Y,
    Z,
    Invalid
};

class AHRS
{
public:
    AHRS(); // 添加默认构造函数声明
    void init();
    void run_once();
    void run_loop(uint16_t loop_ms, bool sendDataAfterRun = false); // 修改 run_loop 函数，添加参数
    void calibration(uint8_t calibration_sensor);
    float getData(SensorType sensorType, Axis axis = Axis::Invalid, float *data = nullptr); // 添加默认参数
    virtual void data_pack(); // 声明 data_pack 方法
    virtual void fetchRawSensorData();    // This function must be overridden to get sensors data.
    virtual void delayMS(uint16_t delayMS); // This function is default Arduino delay(), but can be overridden in a derived class to provide a custom delay implementation.
    virtual void sendData(uint8_t *data, size_t length);

private:
    // Sensor variables
    float accel[3];
    float accel_min[3];
    float accel_max[3];
    float magnetom[3];
    float magnetom_min[3];
    float magnetom_max[3];
    float magnetom_tmp[3];
    float gyro[3];
    float gyro_average[3];
    float eluer[3];
    int16_t gyro_num_samples;
    int16_t raw_accel[3];
    int16_t raw_magnetom[3];
    int16_t raw_gyro[3];
    float real_accel[3];
    float real_magnetom[3];
    float real_gyro[3];
    // DCM variables
    float MAG_Heading;
    float Accel_Vector[3];
    float Gyro_Vector[3];
    float Omega_Vector[3];
    float Omega_P[3];
    float Omega_I[3];
    float Omega[3];
    float errorRollPitch[3];
    float errorYaw[3];
    float DCM_Matrix[3][3];
    float Update_Matrix[3][3];
    float Temporary_Matrix[3][3];

    // Timing variables
    uint32_t timestamp;
    uint32_t timestamp_old;
    float G_Dt;

    // Euler angles
    float yaw;
    float pitch;
    float roll;

    // Private methods
    void resetSensorFusion();
    void compensateSensorErrors();
    void compassHeading();
    void driftCorrection();
    void eulerAngles();
    void initRotationMatrix(float m[3][3], float yaw, float pitch, float roll);
    void matrixMultiply(const float a[3][3], const float b[3][3], float out[3][3]);
    void matrixUpdate();
    void normalize();
    void vectorAdd(float out[3], const float v1[3], const float v2[3]);
    void vectorCrossProduct(float out[3], const float v1[3], const float v2[3]);
    float vectorDotProduct(const float v1[3], const float v2[3]);
    void vectorScale(float out[3], const float v[3], float scale);
    void matrixVectorMultiply(const float a[3][3], const float b[3], float out[3]);
    float lowPassFilter(float currentValue, float previousValue, float alpha);
};

#endif // AHRS_H
