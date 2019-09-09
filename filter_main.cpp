// Arjun Sabnis
// Main code with Kalman filtering for acceleration
// and complementary filter for angle estimation

#include "mbed.h"
#include "rtos.h"
#include "LSM9DS1.h"
#include "LSM9DS1_SPI.h"
#include "kalman.h"

#define PI 3.1415926f

LSM9DS1 lower(p9, p10, 0xD6, 0x3C); // lower arm
SPI imu(p5, p6, p7); // mosi, miso, sclk > for IMU
lsm9ds1_spi upper(imu, p8, p20); // SPI mode, cs_AG, cs_M
//LSM9DS1 upper(p28, p27, 0xD6, 0x3C);// upper arm

// threading stuff
Thread up_arm, low_arm;
Mutex stream;

// timing stuff
Timer t;
volatile float utime, ltime, udt, ldt;

// variables
volatile float u_ax, u_ay, u_az, u_p, u_r, u_y;
volatile float l_ax, l_ay, l_az, l_p, l_r, l_y;
volatile float lZ[3], uZ[3];

// Kalman filters
Kalman ku;
Kalman kl;

// serial stuff
Serial pc(USBTX, USBRX);
Serial BLE(p13, p14);
char upper_buf[128];
char lower_buf[128];

inline void send_buffer(char* buf)
{
    stream.lock();
    BLE.puts(buf);
    stream.unlock();
}

void processLower()
{
    while(1) {
        while(!lower.accelAvailable());
        lower.readAccel();
        l_ax = lower.calcAccel(lower.ax);
        l_ay = lower.calcAccel(lower.ay);
        l_az = lower.calcAccel(lower.az);
        lZ[0] = l_ax;
        lZ[1] = l_ay;
        lZ[2] = l_az;
        kl.update(lZ);

        //while(!lower.gyroAvailable());
        //lower.readGyro();
        //ldt = t.read() - ltime;
        l_p = atan2(-kl.X[0], sqrt(kl.X[1]*kl.X[1] + kl.X[2]*kl.X[2]))*(0.997*180/PI);
        float sign = (kl.X[2]>0)-(kl.X[2]<0);
        l_r = atan2(kl.X[1], sign*sqrt(kl.X[2]*kl.X[2] + 0.1f*kl.X[0]*kl.X[0]))*(180/PI);

        ltime = t.read();
        sprintf(lower_buf,"L,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
                ltime, kl.X[0], kl.X[1], kl.X[2], l_p, l_r, 0);
        //sprintf(lower_buf,"%.3f,%.3f,%.3f\n", l_p, l_r, 0);
        //pc.printf(lower_buf);
        send_buffer(lower_buf);

        Thread::wait(50);
    }
}

void processUpper()
{
    while(1) {
        //while(!upper.accelAvailable());
        //upper.readAccel();
        //u_ax = upper.calcAccel(upper.ax);
        //u_ay = upper.calcAccel(upper.ay);
        //u_az = upper.calcAccel(upper.az);
        upper.read_acc();
        u_ax = upper.accelerometer_data[0];
        u_ay = upper.accelerometer_data[1];
        u_az = upper.accelerometer_data[2];
        uZ[0] = u_ax;
        uZ[1] = u_ay;
        uZ[2] = u_az;
        ku.update(uZ);

        //while(!upper.gyroAvailable());
        //upper.readGyro();
        //upper.read_gyr();
        //udt = t.read() - utime;
        u_p = atan2(-ku.X[0], sqrt(ku.X[1]*ku.X[1] + ku.X[2]*ku.X[2]))*(0.997*180/PI);
        float sign = (ku.X[2]>0)-(ku.X[2]<0);
        u_r = atan2(ku.X[1], sign*sqrt(ku.X[2]*ku.X[2] + 0.1f*ku.X[0]*ku.X[0]))*(180/PI);

        utime = t.read();
        sprintf(upper_buf,"U,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
                utime, ku.X[0], ku.X[1], ku.X[2], u_p, u_r, 0);
        //sprintf(upper_buf,"%.3f,%.3f,%.3f,\n", ku.X[0], ku.X[1], ku.X[2]);
        //pc.printf(upper_buf);
        send_buffer(upper_buf);

        Thread::wait(50);
    }
}

int main()
{
    //pc.baud(9600);
    BLE.baud(9600);

    t.reset();
    utime = ltime = 0.0;

    // calibrate IMUs
    lower.begin();
    lower.calibrate(1);
    kl.initialize();

    upper.init(); // SPI mode
    wait(0.5);
    upper.calibrate();
    ku.initialize();

    t.start();
    low_arm.start(processLower);
    //up_arm.start(processUpper);

    while(1) {}
}

// for debug
/*
void print(volatile float* arr, int r, int c)
{
    for (int i = 0; i < r*c; i++) {
        if (i%c == 0) pc.printf("[ ");
        pc.printf(" %f ", arr[i]);
        if (i%c == (c-1)) pc.printf(" ]\r\n");
    }
}

// old angle functions
//l_r = 0.995f*(l_r + lower.calcGyro(lower.gx)*ldt) + 0.005f*kl.X[0];
//l_p = 0.995*(l_p + lower.calcGyro(lower.gy)*ldt) + 0.005f*kl.X[1];
//l_y = 0.995f*(l_y + lower.calcGyro(lower.gz)*ldt) + 0.005f*kl.X[2];

*/
