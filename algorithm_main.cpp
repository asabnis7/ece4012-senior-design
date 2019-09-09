// Arjun Sabnis
// Main code with Kalman filtering for acceleration, pitch and roll angle
// calculatations, and decision algorithm for good/bad reps

#include "mbed.h"
#include "rtos.h"
#include "LSM9DS1.h"
#include "LSM9DS1_SPI.h"
#include "kalman.h"

#define PI 3.1415926f

BusOut led(LED1,LED2,LED3,LED4);

LSM9DS1 lower(p28, p27, 0xD6, 0x3C); // lower arm
SPI imu(p5, p6, p7);                // mosi, miso, sclk > for IMU
lsm9ds1_spi upper(imu, p8, p20);    // upper arm, SPI mode, cs_AG, cs_M

// thread variables
Thread up_arm, low_arm;
Mutex stream;

// measurements - > x, y, z acceleration
// pitch and roll angles (yaw hard to calculate)
volatile float u_ax, u_ay, u_az, u_p, u_r;  // upper arm
volatile float l_ax, l_ay, l_az, l_p, l_r;  // lower arm
float l_sign, u_sign;

// Kalman filters
Kalman ku;
Kalman kl;
volatile float lZ[3], uZ[3];    // Kalman filter observation vectors

// algorithm variables
volatile float lx_old, lder, uy_base;
volatile float xhigh, xlow, xmid, xmin, xmax;
volatile int high_sample, low_sample, rep_count;
volatile int l_count, u_count;
volatile char curl_prob, lat_prob;
enum REP_DIR {
    UP = 0,
    DOWN = 1,
    REST = 2
};
REP_DIR curl_dir, lat_dir;

// time calculations
Timer t;
volatile float utime, ltime;
volatile float udt, ldt;

// serial stuff
Serial pc(USBTX, USBRX);    // for debug only
Serial BLE(p13, p14);       // bluetooth out
char upper_buf[32];
char lower_buf[32];
char app_msg;    // read from phone app
char out_buf[8];

// decision logic stuff
enum SYSTEM_STATE {
    IDLE = 0,
    START_CURL = 1,
    START_LATS = 2,
    RUNNING = 3,
    STOP = 4,
    CALIBRATE = 5
};
SYSTEM_STATE state;
bool calibrated = false;

// simple send and receive via BLE
inline void send_buffer(char* buf)
{
    stream.lock();
    BLE.puts(buf);
    stream.unlock();
}

inline void receive_msg()
{
    stream.lock();
    if (BLE.readable()) app_msg = BLE.getc();
    stream.unlock();
}

// bicep curl calibration and algorithm
void rep_calibrate()
{
    for (int i = 0; i < 100; i++) {
        while(!lower.accelAvailable());
        lower.readAccel();
        l_ax = lower.calcAccel(lower.ax);

        if ((l_ax < xmin) && (l_ax >= -0.2)) xmin = l_ax;
        if ((l_ax > xmax) && (l_ax <= 1.8)) xmax = l_ax;

        Thread::wait(50);
    }

    calibrated = true;
}

void curl_rep()
{
    ldt = t.read() - ltime;
    lder = (kl.X[0] - lx_old)/ldt;

    if ((lder > 1) && (kl.X[0] > xmid)) curl_dir = UP;
    else if ((lder < -1) && (kl.X[0] < xmid)) curl_dir = DOWN;
    else curl_dir = REST;

    if (curl_dir == UP) {
        led = 0x01;
        if ((lx_old > xhigh) && (kl.X[0] > xhigh)) high_sample++;
    } else if (curl_dir == DOWN) {
        led = 0x02;
        if ((lx_old < xlow) && (kl.X[0] < xlow)) low_sample++;
    }

    if (ku.X[1] < -0.9) u_count++;
    if (l_r < -30.0) l_count++;
    
    if ((high_sample > 5) && (low_sample > 5)) {
        rep_count++;

        if ((l_count <= 4) && (u_count <= 4)) curl_prob = 0;
        else if (u_count > 4)  curl_prob = 'U';   // upper arm is really far out
        else if (l_count > 4) curl_prob = 'L'; // really badly twisted arm

        if (curl_prob != 0)
            sprintf(out_buf,"%d,%c,", rep_count, curl_prob);
        else
            sprintf(out_buf,"%d,", rep_count);
        send_buffer(out_buf);

        high_sample = low_sample = curl_prob = 0;
        l_count = u_count = 0;
        curl_dir = REST;
    }
}

void lat_rep()
{
    ldt = t.read() - ltime;
    lder = (kl.X[0] - lx_old)/ldt;

    if ((lder > 0.5) && (kl.X[0] > xmid)) lat_dir = UP;
    else if ((lder < -0.5) && (kl.X[0] < xmid)) lat_dir = DOWN;
    else lat_dir = REST;

    if (lat_dir == UP) {
        led = 0x01;
        if ((lx_old > xhigh) && (kl.X[0] > xhigh)) high_sample++;
    } else if (lat_dir == DOWN) {
        led = 0x02;
        if ((lx_old < xlow) && (kl.X[0] < xlow)) low_sample++;
    }

    if (ku.X[1] < -0.5) u_count++;   // upper arm is really far out
    if ((kl.X[1] > 0.45) || (ku.X[1] > 0.5)) l_count++;      // really badly twisted arm

    if ((high_sample > 5) && (low_sample > 5)) {
        rep_count++;

        if (l_count <= 4 && u_count <= 4) curl_prob = 0;
        else if (u_count > 4)  lat_prob = 'U';   // upper arm is really far out
        else if (l_count > 4) lat_prob = 'L'; // really badly twisted arm

        if (lat_prob != 0)
            sprintf(out_buf,"%d,%c,", rep_count, lat_prob);
        else
            sprintf(out_buf,"%d,", rep_count);
        send_buffer(out_buf);

        high_sample = low_sample = lat_prob = 0;
        l_count = u_count = 0;
        lat_dir = REST;
    }
}

// upper and lower arm IMU processing thread functions
void processLower()
{
    while(1) {
        // store old X for algorithm
        lx_old = kl.X[0];

        while(!lower.accelAvailable());
        lower.readAccel();
        l_ax = lower.calcAccel(lower.ax);
        l_ay = lower.calcAccel(lower.ay);
        l_az = lower.calcAccel(lower.az);
        lZ[0] = l_ax;
        lZ[1] = l_ay;
        lZ[2] = l_az;
        kl.update(lZ);

        l_p = atan2(-kl.X[0], sqrt(kl.X[1]*kl.X[1] + kl.X[2]*kl.X[2]))*(180/PI);
        l_sign = (kl.X[2]>0)-(kl.X[2]<0);
        l_r = atan2(kl.X[1], l_sign*sqrt(kl.X[2]*kl.X[2] + 0.1f*kl.X[0]*kl.X[0]))*(180/PI);

        //sprintf(lower_buf,"%.3f,%.3f\n", l_r, l_p);
        //send_buffer(lower_buf);

        ltime = t.read();
        Thread::wait(50);
    }
}

void processUpper()
{
    while(1) {
        // store old X for algorithm
        //ux_old = ku.X[0];

        upper.read_acc();
        u_ax = upper.accelerometer_data[0];
        u_ay = upper.accelerometer_data[1];
        u_az = upper.accelerometer_data[2];
        uZ[0] = u_ax;
        uZ[1] = u_ay;
        uZ[2] = u_az;
        ku.update(uZ);

        u_p = atan2(-ku.X[0], sqrt(ku.X[1]*ku.X[1] + ku.X[2]*ku.X[2]))*(180/PI);
        u_sign = (ku.X[2]>0)-(ku.X[2]<0);
        u_r = atan2(ku.X[1], u_sign*sqrt(ku.X[2]*ku.X[2] + 0.1f*ku.X[0]*ku.X[0]))*(180/PI);

        //sprintf(upper_buf,"%.3f,%.3f\n", ku.X[0], ku.X[1]);
        //send_buffer(upper_buf);

        utime = t.read();
        Thread::wait(50);
    }
}


int main()
{
    //pc.baud(9600);
    BLE.baud(9600);

    // calibrate IMUs
    lower.begin();          // forearm
    if (!lower.begin()) {
        BLE.printf("Exit\r\n");
        exit(0);
    }
    lower.calibrate(1);

    upper.init();           // upper arm, SPI mode
    upper.calibrate();

    state = IDLE;

    while(1) {

        if (state == IDLE) {
            // wait for phone signal
            if (BLE.readable()) {
                receive_msg();
                //pc.putc(app_msg);
                if (app_msg == '1') state = START_CURL;
                else if (app_msg == '2') state = START_LATS;
                else if (app_msg == '4') state = CALIBRATE;
                else state = IDLE;
            }
            Thread::wait(250);
        }

        if ((state == START_CURL) || (state == START_LATS)) {
            // reset and start timer
            t.reset();
            utime = ltime = 0.0;
            ldt = udt = lder = 0.0;
            t.start();

            if (state == START_CURL) {
                xmid = ((xmax-xmin)/2) + xmin;
                xhigh = 0.65*xmax;
                xlow = xmid + 0.6*xmin;
            }

            else if (state == START_LATS) {
                xmid = ((xmax-xmin)/2) + xmin;
                xhigh = 0.6*xmax;
                xlow = xmid + 0.7*xmin;
            }

            //BLE.printf("%.3f,%.3f,%.3f,%.3f,%.3f", xmin,xmid,xmax,xlow,xhigh);

            // seed Kalman filters
            kl.initialize();
            ku.initialize();

            Thread::wait(1000);

            // start threads
            low_arm.start(processLower);
            up_arm.start(processUpper);
            state = RUNNING;
        }

        if (state == RUNNING) {
            if (app_msg == '1') curl_rep();
            else if (app_msg == '2') lat_rep();
            if (BLE.readable()) {
                receive_msg();
                if (app_msg == '3') state = STOP;
            }
            Thread::wait(50);
        }

        if (state == STOP) {
            led = 0x00;
            low_arm.terminate();
            up_arm.terminate();
            t.stop();
            //sprintf(out_buf,"stop,");
            //send_buffer(out_buf);
            calibrated = false;
            rep_count = high_sample = low_sample = 0;
            state = IDLE;
        }

        if (state == CALIBRATE) {
            if (calibrated == false) {
                led = 0x01;
                wait(0.75);
                led = 0x02;
                wait(0.75);
                led = 0x04;
                wait(0.75);
                led = 0x08;
                wait(0.75);
                led = 0xFF;
                rep_calibrate();
                led = 0x00;
                //sprintf(out_buf,"0");
                //send_buffer(out_buf);
                state = IDLE;
            } else
                state = IDLE;
        }
    }
}
