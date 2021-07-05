#ifndef SPEED_CONTROLLER_HPP
#define SPEED_CONTROLLER_HPP

#include <Arduino.h>
#include "motor_driver.hpp"
#include "encoder.hpp"

class SpeedController {
    public:
        SpeedController(
            MotorDriver* md_ptr,
            Encoder* enc_ptr,
            double gainP, double gainI, double gainD);

        void setMotorDuty(double duty);
        void setVelocity(double velocity);

        double get_rps() const { return mRPS; };
    private:
        double mP, mI, mD; // PID gain
        double mIntegralSum, mRPS;
        MotorDriver* mMD;
        Encoder* mEncoder;
};

#endif