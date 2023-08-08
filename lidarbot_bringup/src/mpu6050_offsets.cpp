// MPU6050 offsets program

// Adapted from Example.cpp at https://github.com/alex-mous/MPU6050-C-CPP-Library-for-Raspberry-Pi

#include "lidarbot_bringup/mpu6050_lib.h"

MPU6050 device(0x68);

int main() {
	float ax, ay, az, gx, gy, gz; // Variables to store the accel and gyro values

	sleep(1); // Wait for the MPU6050 to stabilize

	// Calculate the offsets
	std::cout << "\nPlease keep the MPU6050 module level and still. This may take a few minutes.\n\n";
	std::cout << "Calculating offsets ...\n\n";
	device.getOffsets(&ax, &ay, &az, &gx, &gy, &gz);
	std::cout << "Gyroscope offsets:\n";
	std::cout << "------------------\n";
	std::cout << "X: " << gx << "\n";
	std::cout << "Y: " << gy << "\n";
	std::cout << "Z: " << gz << "\n\n";

	std::cout << "Accelerometer offsets:\n";
	std::cout << "----------------------\n";
	std::cout << "X: " << ax << "\n";
	std::cout << "Y: " << ay << "\n";
	std::cout << "Z: " << az << "\n\n";

	std::cout << "Include the obtained offsets in the respective macros of the mpu6050_lib.h file.\n";
	
	return 0;
}

