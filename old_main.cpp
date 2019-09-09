// Arjun Sabnis
// Running average implementation

#include "mbed.h"
#include "rtos.h"
#include "LSM9DS1.h"

#define SAMPLES 50
#define PI 3.1415926
#define WINDOW 3
#define WINDOW_F 3.0

LSM9DS1 lower(p9, p10, 0xD6, 0x3C); // lower arm
LSM9DS1 upper(p28, p27, 0xD6, 0x3C);// upper arm

Timer t;
volatile float timestamp;

Serial pc(USBTX, USBRX);
Serial BLE(p13, p14);
char upper_buf[128];
char lower_buf[128];

Thread up_arm, low_arm;
Mutex stream;

//  Values to collect:
//  ax -> x-axis acceleration
//  ay -> y-axis acceleration
//  az -> z-axis acceleration
//  pa -> pitch angle (about y-axis)
//  ra -> roll angle (about x-axis)
//  gz -> z-axis gyro

// lower arm
volatile float l_ax[SAMPLES];
volatile float l_ay[SAMPLES];
volatile float l_az[SAMPLES];
volatile float l_p[SAMPLES];
volatile float l_r[SAMPLES];
volatile float l_gz[SAMPLES];

// upper arm
volatile float u_ax[SAMPLES];
volatile float u_ay[SAMPLES];
volatile float u_az[SAMPLES];
volatile float u_p[SAMPLES];
volatile float u_r[SAMPLES];
volatile float u_gz[SAMPLES];


inline void send_buffer(char* buf)
{
    stream.lock();
    BLE.puts(buf);
    stream.unlock();
}

void collectInitial(LSM9DS1 &IMU, volatile float* x, volatile float* y,
                    volatile float* z, volatile float* p, volatile float* r,
                    volatile float* gz)
{
    float tempx[SAMPLES+WINDOW];
    float tempy[SAMPLES+WINDOW];
    float tempz[SAMPLES+WINDOW];
    float tempgz[SAMPLES+WINDOW];

    for (int i = 0; i < SAMPLES+WINDOW; i++) {
        while(!IMU.accelAvailable());
        IMU.readAccel();
        tempx[i] = IMU.calcAccel(IMU.ax);
        tempy[i] = IMU.calcAccel(IMU.ay);
        tempz[i] = IMU.calcAccel(IMU.az);
        while(!IMU.gyroAvailable());
        IMU.readGyro();
        tempgz[i] = IMU.calcGyro(IMU.gz);
    }

    for (int i = 0; i <= SAMPLES; i++) {
        x[i] = (tempx[i]+tempx[i+1]+tempx[i+2])/WINDOW_F;
        y[i] = (tempy[i]+tempy[i+1]+tempy[i+2])/WINDOW_F;
        z[i] = (tempz[i]+tempz[i+1]+tempz[i+2])/WINDOW_F;
        gz[i] = (tempgz[i]+tempgz[i+1]+tempgz[i+2])/WINDOW_F;
    }

    for (int i = 0; i < SAMPLES; i++) {
        p[i] = atan2(-x[i], sqrt(y[i]*y[i] + z[i]*z[i]))*(180/PI);
        float sign = (z[i]>0)-(z[i]<0);
        r[i] = atan2(y[i], sign*sqrt(z[i]*z[i] + 0.1f*x[i]*x[i]))*(180/PI);
    }
}

void processLower()
{
    while(1) {
        for (int i = 0; i < SAMPLES; i++) {

            if (i < WINDOW) {
                l_ax[i] = l_ax[SAMPLES-WINDOW+i];
                l_ay[i] = l_ay[SAMPLES-WINDOW+i];
                l_az[i] = l_az[SAMPLES-WINDOW+i];
                l_p[i] = l_p[SAMPLES-WINDOW+i];
                l_r[i] = l_r[SAMPLES-WINDOW+i];
                l_gz[i] = l_gz[SAMPLES-WINDOW+i];
            }

            else {
                while(!lower.accelAvailable());
                lower.readAccel();
                l_ax[i] = lower.calcAccel(lower.ax);
                l_ay[i] = lower.calcAccel(lower.ay);
                l_az[i] = lower.calcAccel(lower.az);
                if (abs(l_ax[i]) < 0.001) l_ax[i] = 0.0;
                if (abs(l_ay[i]) < 0.001) l_ay[i] = 0.0;
                if (abs(l_az[i]) < 0.001) l_az[i] = 0.0;
                l_ax[i] = (l_ax[i]+0.8*l_ax[i-1]+0.7*l_ax[i-2])/WINDOW_F;
                l_ay[i] = (l_ay[i]+0.8*l_ay[i-1]+0.7*l_ay[i-2])/WINDOW_F;
                l_az[i] = (l_az[i]+0.8*l_az[i-1]+0.7*l_az[i-2])/WINDOW_F;
                while(!lower.gyroAvailable());
                lower.readGyro();
                l_gz[i] = lower.calcGyro(lower.gz);
                l_gz[i] = (l_gz[i]+0.8*l_gz[i-1]+0.7*l_gz[i-2])/WINDOW_F;

                // stability-adjusted pitch/roll equations
                l_p[i] = atan2(-l_ax[i], sqrt(l_ay[i]*l_ay[i] + l_az[i]*l_az[i]))*(180/PI);
                float sgn = (l_az[i]>0)-(l_az[i]<0);
                l_r[i] = atan2(l_ay[i], sgn*sqrt(l_az[i]*l_az[i] + 0.1f*l_ax[i]*l_ax[i]))*(180/PI);

                timestamp = t.read();
                sprintf(lower_buf,"L,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
                        timestamp, l_ax[i], l_ay[i], l_az[i], l_p[i], l_r[i], l_gz[i]);

                pc.printf(lower_buf);
//                send_buffer(lower_buf);
                Thread::wait(50);
            }
        }
    }
}

void processUpper()
{
    while(1) {
        for (int i = 0; i < SAMPLES; i++) {

            if (i < WINDOW) {
                u_ax[i] = u_ax[SAMPLES-WINDOW+i];
                u_ay[i] = u_ay[SAMPLES-WINDOW+i];
                u_az[i] = u_az[SAMPLES-WINDOW+i];
                u_p[i] = u_p[SAMPLES-WINDOW+i];
                u_r[i] = u_r[SAMPLES-WINDOW+i];
                u_gz[i] = u_gz[SAMPLES-WINDOW+i];
            }

            else {
                while(!upper.accelAvailable());
                upper.readAccel();
                u_ax[i] = upper.calcAccel(upper.ax);
                u_ay[i] = upper.calcAccel(upper.ay);
                u_az[i] = upper.calcAccel(upper.az);
                if (abs(u_ax[i]) < 0.001) u_ax[i] = 0.0;
                if (abs(u_ay[i]) < 0.001) u_ay[i] = 0.0;
                if (abs(u_az[i]) < 0.001) u_az[i] = 0.0;
                u_ax[i] = (u_ax[i]+0.8*u_ax[i-1]+0.7*u_ax[i-2])/WINDOW_F;
                u_ay[i] = (u_ay[i]+0.8*u_ay[i-1]+0.7*u_ay[i-2])/WINDOW_F;
                u_az[i] = (u_az[i]+0.8*u_az[i-1]+0.7*u_az[i-2])/WINDOW_F;
                while(!upper.gyroAvailable());
                upper.readGyro();
                u_gz[i] = upper.calcGyro(upper.gz);
                u_gz[i] = (u_gz[i]+0.8*u_gz[i-1]+0.7*u_gz[i-2])/WINDOW_F;

                // stability-adjusted pitch/roll equations
                u_p[i] = atan2(-u_ax[i], sqrt(u_ay[i]*u_ay[i] + u_az[i]*u_az[i]))*(180/PI);
                float sgn = (u_az[i]>0)-(u_az[i]<0);
                u_r[i] = atan2(u_ay[i], sgn*sqrt(u_az[i]*u_az[i] + 0.1f*u_ax[i]*u_ax[i]))*(180/PI);

                timestamp = t.read();
                sprintf(upper_buf,"U,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
                        timestamp, u_ax[i], u_ay[i], u_az[i], u_p[i], u_r[i], u_gz[i]);
//                    sprintf(upper_buf,"%.3f,%.3f,%.3f\n", u_ax[i], u_ay[i], u_az[i]);
                pc.printf(upper_buf);
//                send_buffer(upper_buf);
                Thread::wait(50);
            }
        }
    }
}

int main()
{
    //pc.baud(9600);
    BLE.baud(9600);
    t.reset();

    // calibrate IMU
    lower.begin();
    if (!lower.begin()) {
//        pc.printf("Failed to communicate with lower LSM9DS1.\n\r");
        exit(0);
    }
    lower.calibrate(1);
    upper.begin();
    if (!upper.begin()) {
//        pc.printf("Failed to communicate with upper LSM9DS1.\n\r");
        exit(0);
    }
    upper.calibrate(1);

    collectInitial(lower, l_ax, l_ay, l_az, l_p, l_r, l_gz);
    collectInitial(upper, u_ax, u_ay, u_az, u_p, u_r, u_gz);
    Thread::wait(500);

    t.start();
    low_arm.start(processLower);
    up_arm.start(processUpper);

    while(1) {}
}
