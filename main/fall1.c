#include <Wire.h>
#include "BMI323.h"

// BMI323 I2C address
#define BMI323_ADDR 0x68

// Quaternion data
struct {
    int16_t q_w;
    int16_t q_x;
    int16_t q_y;
    int16_t q_z;
} quaternion;

// Angular velocity thresholds
#define ANGULAR_VELOCITY_THRESHOLD   5.0f // rad/s
#define ORIENTATION_CHANGE_THRESHOLD 45.0f // degrees

// Gravity threshold
#define GRAVITY_THRESHOLD 0.8f // g

// Fall detection flags
bool fallDetected = false;

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C communication
    Wire.begin();
    
    // Initialize BMI323 sensor
    if (!BMI323.begin(Wire, BMI323_ADDR)) {
        Serial.println("BMI323 initialization failed!");
        while (1);
    }
    
    // Configure sensors
    BMI323.setAccelRange(ACC_RANGE_4G);     // Set accelerometer range to ±4g
    BMI323.setGyroRange(GYRO_RANGE_2000DPS); // Set gyroscope range to ±2000 DPS
    
    // Enable quaternion output (if supported)
    BMI323.enableQuaternion(true);
    
    Serial.println("BMI323 initialized successfully!");
}

void loop() {
    // Read sensor data
    if (!readSensorData()) {
        delay(100);
        return;
    }
    
    // Calculate orientation change
    float angularVelocity = calculateAngularVelocity();
    float orientationChange = calculateOrientationChange();
    
    // Check fall conditions based on quaternion properties
    bool condition1 = (angularVelocity > ANGULAR_VELOCITY_THRESHOLD);          // High angular velocity
    bool condition2 = (orientationChange > ORIENTATION_CHANGE_THRESHOLD);      // Significant orientation change
    bool condition3 = (abs(quaternion.q_w) < GRAVITY_THRESHOLD);              // Loss of vertical gravity component
    bool condition4 = (detectImpact());                                      // Impact detection
    
    // Combined fall detection logic
    if ((condition1 && condition2) || (condition3 && condition4)) {
        if (!fallDetected) {
            Serial.println("Fall detected!");
            fallDetected = true;
            triggerFallAlert();
        }
    } else {
        fallDetected = false;
    }
    
    delay(100);
}

// Read sensor data from BMI323
bool readSensorData() {
    // Read raw quaternion values (if supported)
    if (!BMI323.readQuaternion(&quaternion.q_w, &quaternion.q_x, &quaternion.q_y, &quaternion.q_z)) {
        Serial.println("Failed to read quaternion data!");
        return false;
    }
    
    // Normalize quaternions
    float norm = sqrt(pow(quaternion.q_w, 2) + pow(quaternion.q_x, 2) + pow(quaternion.q_y, 2) + pow(quaternion.q_z, 2));
    if (norm != 0.0f) {
        quaternion.q_w /= norm;
        quaternion.q_x /= norm;
        quaternion.q_y /= norm;
        quaternion.q_z /= norm;
    }
    
    return true;
}

// Calculate angular velocity
float calculateAngularVelocity() {
    // Read gyroscope data
    int16_t gx, gy, gz;
    if (!BMI323.readGyro(&gx, &gy, &gz)) {
        return 0.0f;
    }
    
    // Convert to radians per second
    float angularX = (float)gx * BMI323_GYRO_SENSITIVITY_2000DPS;
    float angularY = (float)gy * BMI323_GYRO_SENSITIVITY_2000DPS;
    float angularZ = (float)gz * BMI323_GYRO_SENSITIVITY_2000DPS;
    
    // Calculate magnitude
    return sqrt(pow(angularX, 2) + pow(angularY, 2) + pow(angularZ, 2));
}

// Calculate orientation change based on quaternions
float calculateOrientationChange() {
    static struct {
        int16_t q_w_prev;
        int16_t q_x_prev;
        int16_t q_y_prev;
        int16_t q_z_prev;
    } prevQuaternion = {0, 0, 0, 0};
    
    // Calculate the difference between current and previous quaternions
    float dq_w = quaternion.q_w - prevQuaternion.q_w;
    float dq_x = quaternion.q_x - prevQuaternion.q_x;
    float dq_y = quaternion.q_y - prev Quaternion.q_y;
    float dq_z = quaternion.q_z - prevQuaternion.q_z;
    
    // Calculate the angle between quaternions (in degrees)
    float angle = 2.0f * atan2(sqrt(pow(dq_x, 2) + pow(dq_y, 2) + pow(dq_z, 2)),
                              abs(dq_w)) * (180.0f / PI);
    
    // Update previous quaternion
    prevQuaternion.q_w = quaternion.q_w;
    prevQuaternion.q_x = quaternion.q_x;
    prevQuaternion.q_y = quaternion.q_y;
    prevQuaternion.q_z = quaternion.q_z;
    
    return angle;
}

// Detect impact using acceleration data
bool detectImpact() {
    int16_t ax, ay, az;
    if (!BMI323.readAccel(&ax, &ay, &az)) {
        return false;
    }
    
    // Convert to g's
    float accX = (float)ax * BMI323_ACCEL_SENSITIVITY_4G;
    float accY = (float)ay * BMI323_ACCEL_SENSITIVITY_4G;
    float accZ = (float)az * BMI323_ACCEL_SENSITIVITY_4G;
    
    // Calculate acceleration magnitude
    float accMag = sqrt(pow(accX, 2) + pow(accY, 2) + pow(accZ, 2));
    
    // Impact detection threshold
    const float IMPACT_THRESHOLD = 1.5f; // g
    
    return (accMag > IMPACT_THRESHOLD);
}

// Trigger fall alert (e.g., send signal to server or activate alarm)
void triggerFallAlert() {
    // Add your fall alert implementation here
    // For example, send data over Wi-Fi or activate an LED
}
