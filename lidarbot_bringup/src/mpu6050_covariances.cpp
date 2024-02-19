// MPU6050 covariance program
// Currently only variances are considered for the imu covariance arrays

#include "lidarbot_bringup/mpu6050_lib.h"

MPU6050 device(0x68);

// Function to find mean
float mean(float array[], int n) {
    float sum = 0.0f;

    for (int i = 0; i < n; i++)
        sum += array[i];
    return sum / n;
}

// Function to find variance
float variance(float array[], int n) {
    float sum = 0.0f;

    float array_mean = mean(array, n); 

    for (int i = 0; i < n; i++)
        sum += (array[i] - array_mean) * (array[i] - array_mean);
    return sum / (n - 1);
}

int main() {

    // Initialize variables and arrays
    float roll, pitch, yaw;                 // Angle values
    float gx, gy, gz;                       // Gyro values 
    float ax, ay, az;                       // Accel values 
    int sample_size = 500;                  // Sample data size to calculate variance on
    float orient_var[3];                    // Orientation variances
    float ang_vel_var[3];                   // Angular velocity variances 
    float lin_accel_var[3];                 // Linear acceleration variances
    float roll_array[sample_size];          // Array of roll values
    float pitch_array[sample_size];         // Array of pitch values
    float yaw_array[sample_size];           // Array of yaw values
    float ang_vel_x_array[sample_size];     // Array of angular velocity values in x-axis
    float ang_vel_y_array[sample_size];     // Array of angular velocity values in y-axis
    float ang_vel_z_array[sample_size];     // Array of angular velocity values in z-axis
    float lin_accel_x_array[sample_size];   // Array of linear acceleration values in x-axis 
    float lin_accel_y_array[sample_size];   // Array of linear acceleration values in y-axis
    float lin_accel_z_array[sample_size];   // Array of linear acceleration values in z-axis

    sleep(1); // Wait for the MPU6050 to stabilize

    device.calc_yaw = true;

    std::cout << "\nPlease keep the MPU6050 module level and still. \n";
    std::cout << "Reading and appending " << sample_size << " sensor data points to respective arrays" << ", it may take a while ... \n\n";

    // Read and append sensor data to arrays
    int i; // loop variable

    for (i = 0; i < sample_size; i++) {
        // Read roll, pitch and yaw angles
        device.getAngle(0, &roll);
        device.getAngle(1, &pitch);
        device.getAngle(2, &yaw);

        roll_array[i] = roll;
        pitch_array[i] = pitch;
        yaw_array[i] = yaw;

        // Read gyro values
        device.getGyro(&gx, &gy, &gz);

        ang_vel_x_array[i] = gx;
        ang_vel_y_array[i] = gy;
        ang_vel_z_array[i] = gz;

        // Read accel values
        device.getAccel(&ax, &ay, &az);

        lin_accel_x_array[i] = ax;
        lin_accel_y_array[i] = ay;
        lin_accel_z_array[i] = az;

        usleep(250000); // 0.25sec delay to read non-consecutive values
    }

    std::cout << "Calculating variances ...\n\n";
    sleep(1);

    // Calculate variances
    orient_var[0] = variance(roll_array, sample_size);
    orient_var[1] = variance(pitch_array, sample_size);
    orient_var[2] = variance(yaw_array, sample_size);
    ang_vel_var[0] = variance(ang_vel_x_array, sample_size);
    ang_vel_var[1] = variance(ang_vel_y_array, sample_size);
    ang_vel_var[2] = variance(ang_vel_z_array, sample_size);
    lin_accel_var[0] = variance(lin_accel_x_array, sample_size);
    lin_accel_var[1] = variance(lin_accel_y_array, sample_size);
    lin_accel_var[2] = variance(lin_accel_z_array, sample_size);

    // Output variance	
    std::cout << "static_covariance_orientation: [" << orient_var[0] << ", 0.0, 0.0, "
            << orient_var[1] << ", 0.0, 0.0, " << orient_var[2] << ", 0.0, 0.0]\n";
    std::cout << "static_covariance_angular_velocity: [" << ang_vel_var[0] << ", 0.0, 0.0, " 
            << ang_vel_var[1] << ", 0.0, 0.0, " << ang_vel_var[2] << ", 0.0, 0.0]\n";
    std::cout << "static_covariance_linear_acceleration: [" << lin_accel_var[0] << ", 0.0, 0.0, " 
            << lin_accel_var[1] << ", 0.0, 0.0, " << lin_accel_var[2] << ", 0.0, 0.0]\n\n";

    std::cout << "Paste covariance arrays in the imu_broadcaster ros__parameters section in lidarbot_bringup/config/controllers.yaml.\n";

    return 0;
}