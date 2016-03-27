# Sound modulation MPU6050

Modulation of a sine wave is achieved interfacing a JUCE console application (TCP server) with the mpu6050 IMU via Particle Photon microcontroller (TCP client)

The raw data of the mpu6050 is noisy on short time scales, and gyroscope data drifts on longer timescales. One way to achieve better data is applying filters. However, the MPU-6050 contains a digital motion processor (DMP) which can perform the data fusion on the IMU chip iteslf.

The IMU is connected with the Particle Photon via I2C. Control over the DMP is achieved via the I2Cdev library. 
Data is sampled at a rate of 100Hz, yaw pitch values are provided from the DMP and sent to the server.

Work in progress
The server receives the data, and feeds it as parameters for the modulation.
