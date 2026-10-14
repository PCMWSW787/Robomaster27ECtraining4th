//
// Created by Siwei Wang on 2026/10/11.
//

#include "PID.hpp"
#include "math.hpp"

PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k)
{
    kp_ = kp; ki_ = ki; kd_ = kd, integration_max_ = i_max, output_current_max_ = out_max, d_filter_k_ = limit(d_filter_k, 0.0f, 1.0f);
}

float PID::calc(float ref, float fdb)
{
    err_ = ref - fdb;
    err_sum_ = limit(err_sum_ + err_, -integration_max_, integration_max_);
    pout_ = kp_ * err_;
    dout_ = kd_ * (err_ - last_err_);
    dout_ = dout_ * (1.0f - d_filter_k_) + last_dout_ * d_filter_k_;
    iout_ = ki_ * err_sum_;
    float output = limit(pout_ + dout_ + iout_, -output_current_max_, output_current_max_);
    last_err_ = err_;
    last_dout_ = dout_;
    return output;
}
void PID::reset()
{
    err_ = 0.0f;
    err_sum_ = 0.0f;
    last_err_ = 0.0f;
    last_dout_ = 0.0f;
}
