#include <Arduino.h>
#include <EEPROM.h>

#include "motor_driver.hpp"
#include "encoder.hpp"
#include "speedController.hpp"

// ports
const byte MD_AIN1 = 7;
const byte MD_AIN2 = 6;
const byte MD_PWMA = 10;
const byte MD_BIN1 = 12;
const byte MD_BIN2 = 13;
const byte MD_PWMB = 11;
const byte ENCODER1_A = 2;
const byte ENCODER1_B = 8;
const byte ENCODER2_A = 3;
const byte ENCODER2_B = 9;

MotorDriver *l_md, *r_md;

Encoder *l_enc, *r_enc;
void l_pulse_callback() { l_enc->pulse_callback(); }
void r_pulse_callback() { r_enc->pulse_callback(); }

SpeedController *spdCtrlL, *spdCtrlR;

unsigned long s_time;

int writeNum = 0;
boolean output_done = false;

void setup()
{
    l_md = new MotorDriver(MD_BIN1, MD_BIN2, MD_PWMB);
    r_md = new MotorDriver(MD_AIN1, MD_AIN2, MD_PWMA);

    l_enc = new Encoder(ENCODER2_A, ENCODER2_B);
    r_enc = new Encoder(ENCODER1_A, ENCODER1_B);

    double gP = 100.0;
    double gI = 2;
    double gD = 0.0;
    spdCtrlL = new SpeedController(l_md, l_enc, gP, gI, gD);
    spdCtrlR = new SpeedController(r_md, r_enc, gP, gI, gD);

    spdCtrlL->setVelocity(0.0);
    spdCtrlR->setVelocity(0.0);

    attachInterrupt(digitalPinToInterrupt(l_enc->mInA), l_pulse_callback, RISING);
    attachInterrupt(digitalPinToInterrupt(r_enc->mInA), r_pulse_callback, RISING);

    s_time = millis();
}

struct datum
{
    short time;
    double rps;
};

int memcnt = 0;  

void loop()
{
    unsigned long utime = millis();
    unsigned long ctime = utime - s_time;

    l_enc->update();
    r_enc->update();

    double vel = 1.0;

    if (ctime <= 2000) {
        spdCtrlL->setVelocity(vel);
        spdCtrlR->setVelocity(vel);
        
        if (memcnt == 10) {
            datum d;
            d.time = ctime;
            d.rps = l_enc->get_rps();
            if (writeNum < 1000) {
                EEPROM.put(writeNum, d);
                writeNum += sizeof(d);
            }
            memcnt = 0;
        }
        memcnt++;
    } else if (ctime <= 4000) {
        spdCtrlL->setVelocity(-vel);
        spdCtrlR->setVelocity(-vel);
    } else {
        spdCtrlL->setMotorDuty(0.0);
        spdCtrlR->setMotorDuty(0.0);

        if (!output_done) {
            output_done = true;
            // print out result
            Serial.begin(9600);
            Serial.println("num of entries" + String(writeNum));

            int outnum = writeNum / sizeof(datum);
            int addr = 0;
            for (int i = 0; i < outnum; ++i)
            {
                datum d;
                EEPROM.get(addr, d);
                addr += sizeof(d);

                Serial.println((String)d.time + " " + (String)d.rps);
            }
            Serial.end();
            
            delete(l_md); delete(r_md);
            delete(l_enc); delete(r_enc);
            delete(spdCtrlL); delete(spdCtrlR);
        }
    }

    unsigned int ftime = utime + 10;
    while(millis() < ftime) {
        //wait approx. [period] ms
    }
}