#ifndef filters_h
#define filters_h
const float beta = 0.41f;
//const float deltat = 0.011;
// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float* sensor_data, float* q, float deltat)
{
    float ax = sensor_data[0], 
    ay = sensor_data[1], 
    az = sensor_data[2], 
    gx = sensor_data[3], 
    gy = sensor_data[4], 
    gz = sensor_data[5], 
    mx = sensor_data[6], 
    my = sensor_data[7], 
    mz = sensor_data[8];
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q0mx;
    float _2q0my;
    float _2q0mz;
    float _2q1mx;
    float _4bx;
    float _4bz;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q0q2 = 2.0f * q0 * q2;
    float _2q2q3 = 2.0f * q2 * q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    // Fixed gradient decent corrective step implementation
//    float _8bx = 2.0f * _4bx;
//    float _8bz = 2.0f * _4bz;
//    s0 = -_2q2 * (2.0f * (q1q3 - q0q2) - ax) + _2q1 * (2.0f * (q0q1 + q2q3) - ay) - _4bz * q2 * (_4bx * (0.5f - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_4bx * q3 + _4bz * q1) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + _4bx * q2 * (_4bx * (q0q2 + q1q3) + _4bz * (0.5f - q1q1 - q2q2) - mz);
//    s1 = _2q3 * (2.0f * (q1q3 - q0q2) - ax) + _2q0 * (2.0f * (q0q1 + q2q3) - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _4bz * q3 * (_4bx * (0.5f - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_4bx * q2 + _4bz * q0) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * q3 - _8bz * q1) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5f - q1q1 - q2q2) - mz); 
//    s2 = -_2q0 * (2.0f * (q1q3 - q0q2) - ax) + _2q3 * (2.0f * (q0q1 + q2q3) - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_8bx * q2 - _4bz * q0) * (_4bx * (0.5f - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_4bx * q1 + _4bz * q3) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * q0 - _8bz * q2) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5f - q1q1 - q2q2) - mz);
//    s3 = _2q1 * (2.0f * (q1q3 - q0q2) - ax) + _2q2 * (2.0f * (q0q1 + q2q3) - ay) + (-_8bx * q3 + _4bz * q1) * (_4bx * (0.5f - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_4bx * q0 + _4bz * q2) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + _4bx * q1 * (_4bx * (q0q2 + q1q3) + _4bz * (0.5f - q1q1 - q2q2) - mz);     

    norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);    // normalise step magnitude
    norm = 1.0f/norm;
    s0 *= norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    // Integrate to yield quaternion
    q0 += qDot1 * deltat;
    q1 += qDot2 * deltat;
    q2 += qDot3 * deltat;
    q3 += qDot4 * deltat;
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q0 * norm;
    q[1] = q1 * norm;
    q[2] = q2 * norm;
    q[3] = q3 * norm;
}

const float Kp = 1.;
const float Ki = 0.0;

 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones. 
void MahonyQuaternionUpdate(float* sensor_data, float* eInt, float* q, float deltat)
{
    float ax = sensor_data[0], 
    ay = sensor_data[1], 
    az = sensor_data[2], 
    gx = sensor_data[3], 
    gy = sensor_data[4], 
    gz = sensor_data[5], 
    mx = sensor_data[6], 
    my = sensor_data[7], 
    mz = sensor_data[8];
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;   

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
    hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
    bx = sqrtf((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
    wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
    wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);  

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0f)
    {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
    }
    else
    {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q1;
    pb = q2;
    pc = q3;
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * (0.5f * deltat);
    q1 = pa + (q0 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q2 = pb + (q0 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q3 = pc + (q0 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    norm = 1.0f / norm;
    q[0] = q0 * norm;
    q[1] = q1 * norm;
    q[2] = q2 * norm;
    q[3] = q3 * norm;
}

#endif