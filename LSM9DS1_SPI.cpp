// Arjun Sabnis
// Calibration function to add to SPI library for IMU
// SPi library: Anaesthetix/LSM9DS1_SPI

/*   
*    Add the following to class declaration
*    void calibrate();
*    bool auto_calc;
*    float gBias[3], aBias[3], mBias[3];
*/

void lsm9ds1_spi::calibrate()
{
    int samples = 50;
    for(int ii = 0; ii < samples; ii++) {
        read_gyr();
        gBias[0] += gyroscope_data[0];
        gBias[1] += gyroscope_data[1];
        gBias[2] += gyroscope_data[2];
        read_acc();
        aBias[0] += accelerometer_data[0];
        aBias[1] += accelerometer_data[1];
        aBias[2] += accelerometer_data[2] - 1.0000; // Assumes sensor facing up!
    }
    for (int ii = 0; ii < 3; ii++) {
        gBias[ii] /= samples;
        aBias[ii] /= samples;
    }

    auto_calc = true;
}
