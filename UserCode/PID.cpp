//
// Created by Siwei Wang on 2026/10/11.
//

#include "PID.hpp"
#include "math.hpp"

PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k)
{
    kp_ = kp; ki_ = ki; kd_ = kd, i_max_ = i_max, out_max_ = out_max, d_filter_k_ = limit(d_filter_k, 0.0f, 1.0f);
}

float PID::calc(float ref, float fdb)
{
    err_ = ref - fdb;
    err_sum_ = limit(err_sum_ + err_, -i_max_, i_max_);
    float pout = kp_ * err_;
    float dout = kd_ * (err_ - last_err_);
    dout = dout * (1.0f - d_filter_k_) + last_dout_ * d_filter_k_;
    float iout = ki_ * err_sum_;
    float output = limit(pout + dout + iout, -out_max_, out_max_);
    last_err_ = err_;
    last_dout_ = dout;
    return output;
}
void PID::reset()
{
    err_ = 0.0f;
    err_sum_ = 0.0f;
    last_err_ = 0.0f;
    last_dout_ = 0.0f;
}
