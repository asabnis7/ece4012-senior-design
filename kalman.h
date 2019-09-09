// Arjun Sabnis
// Kalman filter implementation
// x,y,z acceleration, and possible distance calculation

#include "LSM9DS1.h"

#define g 9.80665f

// states equal to observations
#define st 3    // x, y, z
// #define obs 3   // imu.x, imu.y, imu.z


// Kalman filter for accurate position estimation
struct Kalman {

    volatile float X[st];               // state vector
    //volatile float X_pre[st];           // previous state info
    //volatile float vel[st];             // holds velocity info
    //volatile float vel_pre[st];         // previous velocity
    //volatile float pos[st];             // holds position info

    volatile float P[st*st];            // prediction error covar
    volatile float Q[st*st];            // process noise covar
    volatile float R[st*st];            // measurement error covar

    volatile float G[st*st];            // kalman gain
    volatile float F[st*st];            // process model Jacobian
    volatile float H[st*st];            // measurement model Jacobian

    volatile float fx[st];              // process model output
    volatile float hx[st];              // measurement model output

    volatile float inv[st*st];          // holds inverse values
    volatile float trans[st*st];        // holds transpose values
    volatile float accum1[st*st];       // holds matrix values
    volatile float accum2[st*st];
    volatile float eye[st*st];          // identity matrix
    volatile float vec1[st];            // holds vector values
    volatile float vec2[st];

    void initialize()
    {
        // approximate process noise
        this->Q[0] = 0.0001;
        this->Q[1+1*st] = 0.0001;
        this->Q[2+2*st] = 0.0001;

        /*
        //approximate measurement noise
        float smp = 50;
        float xtemp[(int)smp];
        float ytemp[(int)smp];
        float ztemp[(int)smp];
        float xvar, yvar, zvar, xmean, ymean, zmean;

        // accumulate samples, find measurement noise variance
        for (int i = 0; i < (int)smp; i++) {
            while(!imu.accelAvailable());
            imu.readAccel();
            xtemp[i] = imu.calcAccel(imu.ax);
            ytemp[i] = imu.calcAccel(imu.ay);
            ztemp[i] = imu.calcAccel(imu.az);
            xmean += xtemp[i];
            ymean += ytemp[i];
            zmean += ztemp[i];
        }
        xmean /= smp;
        ymean /= smp;
        zmean /= smp;
        for (int i = 0; i < (int)smp; i++) {
            xvar += pow(xtemp[i]-xmean,2);
            yvar += pow(ytemp[i]-ymean,2);
            zvar += pow(ztemp[i]-zmean,2);
        }
        this-> R[0] = xvar/smp;
        this-> R[1+1*st] = yvar/smp;
        this-> R[2+2*st] = zvar/smp;
        */

        // average variance observed
        this-> R[0] = 0.00001;
        this-> R[1+1*st] = 0.00001;
        this-> R[2+2*st] = 0.00001;

        // process model is simply f(x) = x
        this->fx[0] = this->X[0];
        this->fx[1] = this->X[1];
        this->fx[2] = this->X[2];
        this->F[0] = 1.0;
        this->F[1+1*st] = 1.0;
        this->F[2+2*st] = 1.0;

        // measurement function also simply h(x) = x
        this->hx[0] = this->X[0];
        this->hx[1] = this->X[1];
        this->hx[2] = this->X[2];
        this->H[0] = 1.0;
        this->H[1+1*st] = 1.0;
        this->H[2+2*st] = 1.0;

        // identity matrix setup
        this->eye[0] = 1.0;
        this->eye[1+1*st] = 1.0;
        this->eye[2+2*st] = 1.0;

        // seed p matrix
        this->P[0] = 1.0;
        this->P[1+1*st] = 1.0;
        this->P[2+2*st] = 1.0;

    }

    void transpose(volatile float* arr, volatile float* temp, int sz)
    {
        for (int i = 0; i < sz; i++)
            for (int j = 0; j < sz; j++)
                temp[j*sz+i] = arr[i*sz+j];
    }

    // Matrix inversion using Cholesky decomposition; code modified from: 
    // http://jean-pierre.moreau.pagesperso-orange.fr/Cplus/choles_cpp.txt
    int chol_decomp(int sz, volatile float* arr, float* p)
    {
        float sum;
        int i, j, k;
        // Cholesky decomposition
        for (i = 0; i < sz; i++) {
            for (j = i; j < sz; j++) {
                sum = arr[i*sz+j];
                for (k = i-1; k >= 0; k--) {
                    sum -= arr[i*sz+k] * arr[j*sz+k];
                }
                if (i == j) {
                    if (sum <= 0) return 1;
                    p[i] = sqrt(sum);
                } else arr[j*sz+i] = sum/p[i];
            }
        }

        // Cholesky decomposition inverse
        for (i = 0; i < sz; i++) {
            arr[i*sz+i] = 1/p[i];
            for (j = i+1; j < sz; j++) {
                sum = 0;
                for (k = i; k < j; k++) {
                    sum -= arr[j*sz+k] * arr[k*sz+i];
                }
                arr[j*sz+i] = sum / p[j];
            }
        }
        return 0;
    }

    void inverse(volatile float* arr, volatile float* temp, int sz)
    {
        float p[sz];
        int i, j, k;
        for (i = 0; i < sz; i++)
            for (j = 0; j < sz; j++)
                temp[i*sz+j] = arr[i*sz+j];

        if (chol_decomp(sz, temp, p) == 0) {
            for (i = 0; i < sz; i++) {
                for (j = i+1; j < sz; j++) {
                    temp[i*sz+j] = 0.0;
                }
            }
            for (i = 0; i < sz; i++) {
                temp[i*sz+i] *= temp[i*sz+i];
                for (k = i+1; k < sz; k++) {
                    temp[i*sz+i] += temp[k*sz+i]*temp[k*sz+i];
                }
                for (j = i+1; j < sz; j++) {
                    for (k = j; k < sz; k++) {
                        temp[i*sz+j] += temp[k*sz+i]*temp[k*sz+j];
                    }
                }
            }
            transpose(arr,arr,sz);
        }
    }

    // matrix multiplication: A*B -> C
    // C must be a different array from A and B! will not overwrite itself
    void mult(volatile float* a, volatile float* b, volatile float* c,
              int arow, int acol, int bcol)
    {
        for (int i = 0; i < arow; i++)
            for (int j = 0; j < bcol; j++) {
                c[i*bcol+j] = 0.0;
                for (int m = 0; m < acol; m++)
                    c[i*bcol+j] += a[i*acol+m] * b[m*bcol+j];
            }
    }

    // matrix addition: A+B -> C
    void add(volatile float* a, volatile float* b, volatile float* c,
             int row, int col, int sgn)
    {
        for (int i = 0; i < row; i++)
            for (int j = 0; j < col; j++)
                c[i*col+j] = a[i*col+j] + sgn*b[i*col+j];
    }

    void update(volatile float* Z)
    {
        // h(x) = x
        this->hx[0] = this->X[0];
        this->hx[1] = this->X[1];
        this->hx[2] = this->X[2];

        // P = F * Pk-1 * F^T + Q
        transpose(this->F, this->trans, st);
        mult(this->F, this->P, this->accum1, st, st, st);
        mult(this->accum1, this->trans, this->accum2, st, st, st);
        add(this->accum2, this->Q, this->P, st, st, 1);

        // G = P * H^T(H * P * H^T + R)^-1
        transpose(this->H, this->trans, st);
        mult(this->H, this->P, this->accum1, st, st, st);
        mult(this->accum1, this->trans, this->accum2, st, st, st);
        add(this->accum2, this->R, this->accum1, st, st, 1);
        inverse(this->accum1, this->inv, st);
        mult(this->P, this->trans, this->accum1, st, st, st);
        mult(this->accum1, this->inv, this->G, st, st, st);

        // X = X + G(Z - hx)
        add(Z, this->hx, this->vec1, st, 1, -1);
        mult(this->G, this->vec1, this->vec2, st, st, 1);
        add(this->X, this->vec2, this->X, st, 1, 1);

        // P = (I - G * H)P
        mult(this->G, this->H, this->accum1, st, st, st);
        add(this->eye, this->accum1, this->accum1, st, st, -1);
        mult(this->eye, this->P, this->accum2, st, st, st);
        mult(this->accum1, this->accum2, this->P, st, st, st);
    }

    /*
    void position()
    {
        // add six lines below to update() start if used
        // previous X storage for position
        if (abs(this->X[0]) < 0.001) X_pre[0] = 0.0;
        this->X_pre[0] = this->X[0];
        if (abs(this->X[1]) < 0.001) X_pre[1] = 0.0;
        this->X_pre[1] = this->X[1];
        if (abs(this->X[2]) < 0.001) X_pre[2] = 0.0;
        this->X_pre[2] = this->X[2];
        
        // save old velocity
        this->vel_pre[0] = this->vel[0];
        this->vel_pre[1] = this->vel[1];
        this->vel_pre[2] = this->vel[2];

        // integration of acceleration -> velocity
        this->vel[0] += (this->X[0] - this->X_pre[0])*g; 
        this->vel[1] += (this->X[1] - this->X_pre[1])*g;
        this->vel[2] += (this->X[2] - this->X_pre[2])*g;
        
        // integration of velocity -> position
        this->pos[0] += (this->vel[0] - this->vel_pre[0]); 
        this->pos[1] += (this->vel[1] - this->vel_pre[1]);
        this->pos[2] += (this->vel[2] - this->vel_pre[2]);
    }*/

};
