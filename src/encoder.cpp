#include "encoder.hpp"

Encoder::Encoder(byte inA, byte inB) :
    mInA(inA),
    mInB(inB),
    mCnt(0.0),
    mLastCnt(0.0),
    mLastTime(0.0),
    mRPS(0.0)
{
    pinMode(mInA, INPUT_PULLUP);
    pinMode(mInB, INPUT_PULLUP);
}

void Encoder::pulse_callback()
{
    if (digitalRead(mInB) == 0) {
        mCnt++;
    } else {
        mCnt--;
    }
}

void Encoder::update()
{
    double c_cnt = mCnt, mCnt = 0;
    double ctime = millis();
    mDt = ctime - mLastTime;

    if (mDt == 0.0)
        mRPS = 0.0;
    else
        mRPS = (c_cnt - mLastCnt) / 600.0 * (1000.0 / mDt);

    mLastCnt = c_cnt;
    mLastTime = ctime;
}

