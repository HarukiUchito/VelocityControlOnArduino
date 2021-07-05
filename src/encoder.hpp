#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

class Encoder {
    public:
        Encoder(byte inA, byte inB);
        int get_count() const { return mCnt; };
        double get_rps() const { return mRPS; };
        void update();
        double get_dt() const { return mDt; };

        void pulse_callback();

        byte mInA;
        double mCnt;
        double mLastCnt;
    private:
        byte mInB;
        
        double mLastTime, mDt, mRPS;
};

#endif