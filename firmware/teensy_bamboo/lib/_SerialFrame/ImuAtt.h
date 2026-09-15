#ifndef IMU_ATT_H
#define IMU_ATT_H

// Helper d'attitude IMU pour la branche trames binaires du Teensy (header-only).
// Possede le driver BRUT MPU6050 (pas le wrapper ROS lib/imu/default_imu.h, qui tire
// *_msgs / micro_ros_utilities) et calcule roll/pitch/yaw par un FILTRE COMPLEMENTAIRE :
//   - roll/pitch ABSOLUS depuis l'accelerometre (gravite) ;
//   - yaw RELATIF depuis l'integration du gyro-z.
// Meme approche que le fix ESP32 (cf. memoire esp32-imu-orientation-missing) : sans ce
// calcul, emitImu() n'emettrait que des zeros (le MPU6050 ne fournit pas d'orientation).
//
// ATTENTION : le MPU6050 n'a PAS de magnetometre -> le yaw est RELATIF et DERIVE dans le
// temps (pas de nord absolu). C'est un caveat documente, pas un bug.
//
// N'inclut JAMAIS imu.h / imu_interface.h / default_imu.h / odometry.h.

#include <Arduino.h>
#include <math.h>
#include "MPU6050.h"

class ImuAtt {
  public:
    // Echelles MPU6050 par defaut (identiques a MPU6050IMU dans default_imu.h).
    // accel : +-2 g -> 16384 LSB/g ; gyro : +-250 deg/s -> 131 LSB/(deg/s).
    bool begin() {
        Wire.begin();
        mpu_.initialize();
        if (!mpu_.testConnection()) return false;
        lastUs_ = micros();
        return true;
    }

    // Lit le capteur et met a jour l'attitude. A appeler a cadence reguliere (~10 Hz).
    void read(float& roll, float& pitch, float& yaw) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu_.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // dt reel entre deux lectures (integration gyro).
        uint32_t now = micros();
        float dt = (now - lastUs_) * 1.0e-6f;
        lastUs_ = now;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.0f; // garde-fou (premiere iter / overflow)

        // Accelerometre -> roll/pitch absolus (unite g ; le ratio suffit, pas de mise a l'echelle).
        const float axf = (float)ax, ayf = (float)ay, azf = (float)az;
        const float rollAcc  = atan2f(ayf, azf);
        const float pitchAcc = atan2f(-axf, sqrtf(ayf * ayf + azf * azf));

        // Gyro -> rad/s (deg/s puis DEG_TO_RAD).
        const float gyro_scale = (1.0f / 131.0f) * (float)DEG_TO_RAD;
        const float gxr = gx * gyro_scale;
        const float gyr = gy * gyro_scale;
        const float gzr = gz * gyro_scale;

        // Filtre complementaire : gyro (haute freq) + accel (basse freq) sur roll/pitch.
        const float alpha = 0.98f;
        roll_  = alpha * (roll_  + gxr * dt) + (1.0f - alpha) * rollAcc;
        pitch_ = alpha * (pitch_ + gyr * dt) + (1.0f - alpha) * pitchAcc;
        // Yaw : integration pure du gyro-z (RELATIF, derive -- pas de magneto).
        yaw_  += gzr * dt;

        roll = roll_; pitch = pitch_; yaw = yaw_;
    }

  private:
    MPU6050  mpu_;
    float    roll_ = 0, pitch_ = 0, yaw_ = 0;
    uint32_t lastUs_ = 0;
};

#endif // IMU_ATT_H
