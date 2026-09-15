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
// Le biais du gyro (offset non nul a l'arret) est CALIBRE au begin() : moyenne des
// echantillons gyro carte immobile, soustraite ensuite a chaque lecture. Reduit fortement
// la derive du yaw (sinon integration d'un offset -> saturation ±187.7° observee). Un
// garde-fou rejette la calibration si la carte bouge pendant l'echantillonnage (biais=0).
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
        calibrateGyro();
        lastUs_ = micros();
        return true;
    }

    // Calibration du biais gyro : moyenne de N echantillons carte IMMOBILE.
    // Rejette la calibration si le gyro bouge trop pendant l'echantillonnage (pic-a-pic
    // au-dela d'un seuil) -> biais laisse a 0 plutot que de figer un mauvais offset.
    // Renvoie true si la calibration a ete retenue.
    bool calibrateGyro(uint16_t samples = 200, float maxSpanLsb = 200.0f) {
        int16_t ax, ay, az, gx, gy, gz;
        float sx = 0, sy = 0, sz = 0;
        int16_t minx = 32767, maxx = -32768, miny = 32767, maxy = -32768,
                minz = 32767, maxz = -32768;
        for (uint16_t i = 0; i < samples; i++) {
            mpu_.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            sx += gx; sy += gy; sz += gz;
            if (gx < minx) minx = gx; if (gx > maxx) maxx = gx;
            if (gy < miny) miny = gy; if (gy > maxy) maxy = gy;
            if (gz < minz) minz = gz; if (gz > maxz) maxz = gz;
            delay(2); // ~200 ech. * 2 ms = ~0.4 s d'echantillonnage
        }
        // Carte en mouvement pendant la calib -> etendue trop grande, on rejette.
        if ((maxx - minx) > maxSpanLsb || (maxy - miny) > maxSpanLsb ||
            (maxz - minz) > maxSpanLsb) {
            gxBias_ = gyBias_ = gzBias_ = 0.0f;
            return false;
        }
        gxBias_ = sx / samples;
        gyBias_ = sy / samples;
        gzBias_ = sz / samples;
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

        // Gyro -> rad/s (biais retire, puis deg/s puis DEG_TO_RAD).
        const float gyro_scale = (1.0f / 131.0f) * (float)DEG_TO_RAD;
        const float gxr = (gx - gxBias_) * gyro_scale;
        const float gyr = (gy - gyBias_) * gyro_scale;
        const float gzr = (gz - gzBias_) * gyro_scale;

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
    float    gxBias_ = 0, gyBias_ = 0, gzBias_ = 0; // biais gyro (LSB) mesures au begin()
    uint32_t lastUs_ = 0;
};

#endif // IMU_ATT_H
