#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

#include "motor_driver.hpp"
#include "encoder.hpp"
#include "speedController.hpp"

typedef union float_t {
    int32_t data_i32;
    float data_f32;
} float_t;

class FloatSender {
    public:
        FloatSender() {
            data_.data_f32 = 0.0;
        }

        void send(float n) {
            memset(float_buf_, 0, 4);
            data_.data_f32 = n;
            float_buf_[0] = (uint8_t)((data_.data_i32 & 0xFF000000) >> 24);
            float_buf_[1] = (uint8_t)((data_.data_i32 & 0x00FF0000) >> 16);
            float_buf_[2] = (uint8_t)((data_.data_i32 & 0x0000FF00) >>  8);
            float_buf_[3] = (uint8_t)((data_.data_i32 & 0x000000FF) >>  0);

            for (int i = 0; i < 4; ++i)
                Wire.write(float_buf_[i]);
        }
    private:        
        float_t data_;
        uint8_t float_buf_[4];
};

const int RCV_BUF_MAX = 128;
class I2CReciever {
    public:
        I2CReciever() : index_(0) {
            memset(float_buf_, 0, RCV_BUF_MAX);
        }

        void recieve() {
            index_ = 0;
            while (Wire.available())
                float_buf_[index_++] = Wire.read();
        }

        float read_float_from_buf(int idx) {
            float_t data;
            data.data_i32 = 0.0;
            data.data_i32 |= ((uint32_t)float_buf_[idx + 0] << 24);
            data.data_i32 |= ((uint32_t)float_buf_[idx + 1] << 16);
            data.data_i32 |= ((uint32_t)float_buf_[idx + 2] <<  8);
            data.data_i32 |= ((uint32_t)float_buf_[idx + 3] <<  0);
            return data.data_f32;
        }

        int size() const { return index_; }
        uint8_t get_byte(int index) { return float_buf_[index]; }
    private:
        int index_;
        uint8_t float_buf_[RCV_BUF_MAX];
};

FloatSender fs;
I2CReciever i2cRcv;
float vel_lin = 40.2;
float vel_ang = 1000;

void requestEvent() {

    fs.send(vel_lin);
    vel_ang += 0.1;
    fs.send(vel_ang);
    vel_lin -= 1.0;
}

void receiveEvent(int howMany)
{
    i2cRcv.recieve();
}

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

double target_linear_vel = 0.0;
double target_angular_vel = 0.0;

void setup() {
    Wire.begin(8);// Slave ID #8
    pinMode(SDA, INPUT);
    pinMode(SCL, INPUT);
    
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    // prepare motor control instances
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
    
    Serial.begin(9600);
}

void loop() {
    unsigned long utime = millis();
    //Serial.println(cnt);         // print the integer
    unsigned int ftime = utime + 10;

    int n = i2cRcv.size();
    if (n > 1) {
        //for (int i = 0; i < n; ++i) Serial.print(String(i2cRcv.get_byte(i)) + ", "); Serial.println();
        target_linear_vel = i2cRcv.read_float_from_buf(1);
        target_angular_vel = i2cRcv.read_float_from_buf(5);
        Serial.println(String(target_linear_vel) + ", " + String(target_angular_vel));

        double tread = 0.31;
        double vr = target_linear_vel + tread * target_angular_vel / 2.0;
        double vl = target_linear_vel - tread * target_angular_vel / 2.0;

        spdCtrlL->setVelocity(vl);
        spdCtrlR->setVelocity(vr);
    }

    l_enc->update();
    r_enc->update();
    
    //Serial.println(String(spdCtrlL->get_velocity()));
    spdCtrlL->controlVelocity();
    spdCtrlR->controlVelocity();

    while(millis() < ftime) {
        //wait approx. [period] ms
    }
}
