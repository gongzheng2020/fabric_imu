#include "AHRS.h"
#include "math.h"
#include "stdlib.h"
#include "Arduino.h"

// Constructor
AHRS::AHRS()
    : real_accel{0}, real_gyro{0}, real_magnetom{0}, accel{0}, accel_min{0}, accel_max{0}, magnetom{0}, magnetom_min{0}, magnetom_max{0}, magnetom_tmp{0},
      gyro{0}, gyro_average{0}, gyro_num_samples(0), MAG_Heading(0), Accel_Vector{0}, Gyro_Vector{0},
      Omega_Vector{0}, Omega_P{0}, Omega_I{0}, Omega{0}, errorRollPitch{0}, errorYaw{0},
      DCM_Matrix{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, Update_Matrix{{0, 1, 2}, {3, 4, 5}, {6, 7, 8}},
      Temporary_Matrix{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, timestamp(0), timestamp_old(0), G_Dt(0),
      yaw(0), pitch(0), roll(0) {}

// Public methods
void AHRS::init()
{
    fetchRawSensorData();
    resetSensorFusion();
}

void AHRS::run_once()
{
    fetchRawSensorData();
    compensateSensorErrors();
    compassHeading();
    matrixUpdate();
    normalize();
    driftCorrection();
    eulerAngles();
}

void AHRS::data_pack()
{
    int16_t _cnt = 0;
    uint8_t data_to_send[100]; // 待发送的字节数组
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x55;
    uint8_t _start = _cnt;

    // 发送 Euler 数据
    float eulerData[3];
    getData(SensorType::Euler, Axis::Invalid, eulerData);
    for (int i = 0; i < 3; i++) {
        FloatToBytes converter;
        converter.f = eulerData[i];
        data_to_send[_cnt++] = converter.b[3];
        data_to_send[_cnt++] = converter.b[2];
        data_to_send[_cnt++] = converter.b[1];
        data_to_send[_cnt++] = converter.b[0];
    }

    // 发送 Accel 数据
    float accelData[3];
    getData(SensorType::Accel, Axis::Invalid, accelData);
    for (int i = 0; i < 3; i++) {
        FloatToBytes converter;
        converter.f = accelData[i];
        data_to_send[_cnt++] = converter.b[3];
        data_to_send[_cnt++] = converter.b[2];
        data_to_send[_cnt++] = converter.b[1];
        data_to_send[_cnt++] = converter.b[0];
    }

    // 发送 Gyro 数据
    float gyroData[3];
    getData(SensorType::Gyro, Axis::Invalid, gyroData);
    for (int i = 0; i < 3; i++) {
        FloatToBytes converter;
        converter.f = gyroData[i];
        data_to_send[_cnt++] = converter.b[3];
        data_to_send[_cnt++] = converter.b[2];
        data_to_send[_cnt++] = converter.b[1];
        data_to_send[_cnt++] = converter.b[0];
    }

    // 添加校验码
    uint8_t checksum = 0;
    for (int i = _start; i < _cnt; i++) {
        checksum += data_to_send[i];
    }
    data_to_send[_cnt++] = checksum;

    // 调用虚函数发送数据
    sendData(data_to_send, _cnt);
}

void AHRS::run_loop(uint16_t loop_ms, bool sendDataAfterRun)
{
    if ((millis() - timestamp) >= loop_ms)
    {
        timestamp_old = timestamp;
        timestamp = millis();
        if (timestamp > timestamp_old)
            G_Dt = (float)(timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
        else
            G_Dt = 0;
        run_once();
        if (sendDataAfterRun) {
            data_pack(); // 如果参数为 true，则调用 data_pack
        }
    }
}

// Private methods
void AHRS::resetSensorFusion()
{
    float temp1[3];
    float temp2[3];
    float xAxis[] = {1.0f, 0.0f, 0.0f};

    pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
    vectorCrossProduct(temp1, accel, xAxis);
    vectorCrossProduct(temp2, xAxis, temp1);
    roll = atan2(temp2[1], temp2[2]);
    compassHeading();
    yaw = MAG_Heading;
    initRotationMatrix(DCM_Matrix, yaw, pitch, roll);
}

void AHRS::compensateSensorErrors()
{
    accel[0] = (raw_accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (raw_accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (raw_accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
    real_accel[0] = accel[0]/GRAVITY;
    real_accel[1] = accel[1]/GRAVITY;
    real_accel[2] = accel[2]/GRAVITY;

#ifdef CALIBRATION__MAGN_USE_EXTENDED
    for (int i = 0; i < 3; i++)
        magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    matrixVectorMultiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
    magnetom[0] = (raw_magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magnetom[1] = (raw_magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magnetom[2] = (raw_magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
    real_magnetom[0] = magnetom[0] / MAGN_SCALED_GAUSS(1);
    real_magnetom[1] = magnetom[1] / MAGN_SCALED_GAUSS(1);
    real_magnetom[2] = magnetom[2] / MAGN_SCALED_GAUSS(1);
#endif

    gyro[0] = raw_gyro[0] - GYRO_AVERAGE_OFFSET_X; // gyro x roll
    gyro[1] = raw_gyro[1] - GYRO_AVERAGE_OFFSET_Y; // gyro y pitch
    gyro[2] = raw_gyro[2] - GYRO_AVERAGE_OFFSET_Z; // gyro z yaw
    real_gyro[0] = gyro[0] * GYRO_SCALED_RAD(1);
    real_gyro[1] = gyro[1] * GYRO_SCALED_RAD(1);
    real_gyro[2] = gyro[2] * GYRO_SCALED_RAD(1);
}

void AHRS::compassHeading()
{
    float mag_x;
    float mag_y;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;

    cos_roll = cos(roll);
    sin_roll = sin(roll);
    cos_pitch = cos(pitch);
    sin_pitch = sin(pitch);

    // Tilt compensated magnetic field X
    mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
    // Tilt compensated magnetic field Y
    mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
    // Magnetic Heading
    MAG_Heading = atan2(-mag_y, mag_x);
}

void AHRS::delayMS(uint16_t delayMS)
{
    // This function is empty, but can be overridden in a derived class
    // to provide a custom delay implementation.
    delay(delayMS);
}

float AHRS::getData(SensorType sensorType, Axis axis, float *data)
{
    if (axis != Axis::Invalid)
    {
        switch (sensorType)
        {
        case SensorType::Accel:
            return real_accel[static_cast<int>(axis)];
        case SensorType::Magnetom:
            return real_magnetom[static_cast<int>(axis)];
        case SensorType::Gyro:
            return real_gyro[static_cast<int>(axis)];
        case SensorType::Euler:
            return eluer[static_cast<int>(axis)];
        default:
            return 0.0f;
        }
    }
    else if (data)
    {
        const float *source = nullptr;
        switch (sensorType)
        {
        case SensorType::Accel:
            source = real_accel;
            break;
        case SensorType::Magnetom:
            source = real_magnetom;
            break;
        case SensorType::Gyro:
            source = real_gyro;
            break;
        case SensorType::Euler:
            source = eluer;
            break;
        default:
            std::fill(data, data + 3, 0.0f);
            return 0.0f;
        }
        std::copy(source, source + 3, data);
    }
    return 0.0f;
}

void AHRS::calibration(uint8_t calibration_sensor)
{
    fetchRawSensorData();
    if (calibration_sensor == 0) // Accelerometer
    {
        // Output MIN/MAX values
        Serial.print("accel x,y,z (min/max) = ");
        for (int i = 0; i < 3; i++)
        {
            if (raw_accel[i] < accel_min[i])
                accel_min[i] = raw_accel[i];
            if (raw_accel[i] > accel_max[i])
                accel_max[i] = raw_accel[i];
            Serial.print(accel_min[i]);
            Serial.print("/");
            Serial.print(accel_max[i]);
            if (i < 2)
                Serial.print("  ");
            else
                Serial.println();
        }
    }
    else if (calibration_sensor == 1) // Magnetometer
    {
        // Output MIN/MAX values
        Serial.print("magn x,y,z (min/max) = ");
        for (int i = 0; i < 3; i++)
        {
            if (raw_magnetom[i] < magnetom_min[i])
                magnetom_min[i] = raw_magnetom[i];
            if (raw_magnetom[i] > magnetom_max[i])
                magnetom_max[i] = raw_magnetom[i];
            Serial.print(magnetom_min[i]);
            Serial.print("/");
            Serial.print(magnetom_max[i]);
            if (i < 2)
                Serial.print("  ");
            else
                Serial.println();
        }
    }
    else if (calibration_sensor == 2) // Gyroscope
    {
        // Average gyro values
        for (int i = 0; i < 3; i++)
            gyro_average[i] += raw_gyro[i];
        gyro_num_samples++;

        // Output current and averaged gyroscope values
        Serial.print("gyro x,y,z (current/average) = ");
        for (int i = 0; i < 3; i++)
        {
            Serial.print(raw_gyro[i]);
            Serial.print("/");
            Serial.print(gyro_average[i] / (float)gyro_num_samples);
            if (i < 2)
                Serial.print("  ");
            else
                Serial.println();
        }
    }
    delayMS(100);
}

void AHRS::normalize()
{
    float error = 0;
    float temporary[3][3];
    float renorm = 0;

    error = -vectorDotProduct(&DCM_Matrix[0][0], &DCM_Matrix[1][0]) * .5; // eq.19

    vectorScale(&temporary[0][0], &DCM_Matrix[1][0], error); // eq.19
    vectorScale(&temporary[1][0], &DCM_Matrix[0][0], error); // eq.19

    vectorAdd(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]); // eq.19
    vectorAdd(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]); // eq.19

    vectorCrossProduct(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b //eq.20

    renorm = .5 * (3 - vectorDotProduct(&temporary[0][0], &temporary[0][0])); // eq.21
    vectorScale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

    renorm = .5 * (3 - vectorDotProduct(&temporary[1][0], &temporary[1][0])); // eq.21
    vectorScale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

    renorm = .5 * (3 - vectorDotProduct(&temporary[2][0], &temporary[2][0])); // eq.21
    vectorScale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

void AHRS::driftCorrection()
{
    float mag_heading_x;
    float mag_heading_y;
    float errorCourse;
    // Compensation the Roll, Pitch and Yaw drift.
    static float Scaled_Omega_P[3];
    static float Scaled_Omega_I[3];
    float Accel_magnitude;
    float Accel_weight;

    //*****Roll and Pitch***************

    // Calculate the magnitude of the accelerometer vector
    Accel_magnitude = sqrt(Accel_Vector[0] * Accel_Vector[0] + Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]);
    Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
    // Dynamic weighting of accelerometer info (reliability filter)
    // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
    Accel_weight = constrain(1 - 2 * abs(1 - Accel_magnitude), 0, 1); //

    vectorCrossProduct(&errorRollPitch[0], &Accel_Vector[0], &DCM_Matrix[2][0]); // adjust the ground of reference
    vectorScale(&Omega_P[0], &errorRollPitch[0], Kp_ROLLPITCH * Accel_weight);

    vectorScale(&Scaled_Omega_I[0], &errorRollPitch[0], Ki_ROLLPITCH * Accel_weight);
    vectorAdd(Omega_I, Omega_I, Scaled_Omega_I);

    //*****YAW***************
    // We make the gyro YAW drift correction based on compass magnetic heading

    mag_heading_x = cos(MAG_Heading);
    mag_heading_y = sin(MAG_Heading);
    errorCourse = (DCM_Matrix[0][0] * mag_heading_y) - (DCM_Matrix[1][0] * mag_heading_x); // Calculating YAW error
    vectorScale(errorYaw, &DCM_Matrix[2][0], errorCourse);                                 // Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

    vectorScale(&Scaled_Omega_P[0], &errorYaw[0], Kp_YAW); //.01proportional of YAW.
    vectorAdd(Omega_P, Omega_P, Scaled_Omega_P);           // Adding  Proportional.

    vectorScale(&Scaled_Omega_I[0], &errorYaw[0], Ki_YAW); //.00001Integrator
    vectorAdd(Omega_I, Omega_I, Scaled_Omega_I);           // adding integrator to the Omega_I
}

void AHRS::matrixUpdate()
{
    Gyro_Vector[0] = GYRO_SCALED_RAD(gyro[0]); // gyro x roll
    Gyro_Vector[1] = GYRO_SCALED_RAD(gyro[1]); // gyro y pitch
    Gyro_Vector[2] = GYRO_SCALED_RAD(gyro[2]); // gyro z yaw

    Accel_Vector[0] = accel[0];
    Accel_Vector[1] = accel[1];
    Accel_Vector[2] = accel[2];

    vectorAdd(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  // adding proportional term
    vectorAdd(&Omega_Vector[0], &Omega[0], &Omega_P[0]); // adding Integrator term

#if DEBUG_DRIFT_CORRECTION // Do not use drift correction
    Update_Matrix[0][0] = 0;
    Update_Matrix[0][1] = -G_Dt * Gyro_Vector[2]; //-z
    Update_Matrix[0][2] = G_Dt * Gyro_Vector[1];  // y
    Update_Matrix[1][0] = G_Dt * Gyro_Vector[2];  // z
    Update_Matrix[1][1] = 0;
    Update_Matrix[1][2] = -G_Dt * Gyro_Vector[0];
    Update_Matrix[2][0] = -G_Dt * Gyro_Vector[1];
    Update_Matrix[2][1] = G_Dt * Gyro_Vector[0];
    Update_Matrix[2][2] = 0;
#else // Use drift correction
    Update_Matrix[0][0] = 0;
    Update_Matrix[0][1] = -G_Dt * Omega_Vector[2]; //-z
    Update_Matrix[0][2] = G_Dt * Omega_Vector[1];  // y
    Update_Matrix[1][0] = G_Dt * Omega_Vector[2];  // z
    Update_Matrix[1][1] = 0;
    Update_Matrix[1][2] = -G_Dt * Omega_Vector[0]; //-x
    Update_Matrix[2][0] = -G_Dt * Omega_Vector[1]; //-y
    Update_Matrix[2][1] = G_Dt * Omega_Vector[0];  // x
    Update_Matrix[2][2] = 0;
#endif

    matrixMultiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); // a*b=c

    for (int x = 0; x < 3; x++) // Matrix Addition (update)
    {
        for (int y = 0; y < 3; y++)
        {
            DCM_Matrix[x][y] += Temporary_Matrix[x][y];
        }
    }
}

// Low-pass filter implementation
float AHRS::lowPassFilter(float currentValue, float previousValue, float alpha) {
    return alpha * currentValue + (1 - alpha) * previousValue;
}

void AHRS::eulerAngles()
{
    pitch = -asin(DCM_Matrix[2][0]);
    roll = atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);
    yaw = atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]);

    // Apply low-pass filter to Euler angles
    static float filteredPitch = 0;
    static float filteredRoll = 0;
    static float filteredYaw = 0;
    const float alpha = 0.1f; // Filter coefficient (adjust as needed)

    filteredPitch = lowPassFilter(pitch, filteredPitch, alpha);
    filteredRoll = lowPassFilter(roll, filteredRoll, alpha);
    filteredYaw = lowPassFilter(yaw, filteredYaw, alpha);

    eluer[0] = filteredPitch;
    eluer[1] = filteredRoll;
    eluer[2] = filteredYaw;
}

float AHRS::vectorDotProduct(const float v1[3], const float v2[3])
{
    float result = 0;

    for (int c = 0; c < 3; c++)
    {
        result += v1[c] * v2[c];
    }

    return result;
}

// Computes the cross product of two vectors
// out has to different from v1 and v2 (no in-place)!
void AHRS::vectorCrossProduct(float out[3], const float v1[3], const float v2[3])
{
    out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
    out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
    out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

// Multiply the vector by a scalar
void AHRS::vectorScale(float out[3], const float v[3], float scale)
{
    for (int c = 0; c < 3; c++)
    {
        out[c] = v[c] * scale;
    }
}

// Adds two vectors
void AHRS::vectorAdd(float out[3], const float v1[3], const float v2[3])
{
    for (int c = 0; c < 3; c++)
    {
        out[c] = v1[c] + v2[c];
    }
}

// Multiply two 3x3 matrices: out = a * b
// out has to different from a and b (no in-place)!
void AHRS::matrixMultiply(const float a[3][3], const float b[3][3], float out[3][3])
{
    for (int x = 0; x < 3; x++) // rows
    {
        for (int y = 0; y < 3; y++) // columns
        {
            out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
        }
    }
}

// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void AHRS::matrixVectorMultiply(const float a[3][3], const float b[3], float out[3])
{
    for (int x = 0; x < 3; x++)
    {
        out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
    }
}

// Init rotation matrix using euler angles
void AHRS::initRotationMatrix(float m[3][3], float yaw, float pitch, float roll)
{
    float c1 = cos(roll);
    float s1 = sin(roll);
    float c2 = cos(pitch);
    float s2 = sin(pitch);
    float c3 = cos(yaw);
    float s3 = sin(yaw);

    // Euler angles, right-handed, intrinsic, XYZ convention
    // (which means: rotate around body axes Z, Y', X'')
    m[0][0] = c2 * c3;
    m[0][1] = c3 * s1 * s2 - c1 * s3;
    m[0][2] = s1 * s3 + c1 * c3 * s2;

    m[1][0] = c2 * s3;
    m[1][1] = c1 * c3 + s1 * s2 * s3;
    m[1][2] = c1 * s2 * s3 - c3 * s1;

    m[2][0] = -s2;
    m[2][1] = c2 * s1;
    m[2][2] = c1 * c2;
}