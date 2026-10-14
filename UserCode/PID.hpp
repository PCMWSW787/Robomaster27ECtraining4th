//
// Created by Siwei Wang on 2026/10/11.
//

#ifndef ROBOMASTER27ECTRAINING4TH_PID_HPP
#define ROBOMASTER27ECTRAINING4TH_PID_HPP


class PID
{
public:
    PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k);
    void reset();
    float calc(float ref, float fdb);
private:
    float kp_, ki_, kd_;
    float i_max_, out_max_, d_filter_k_;
    float err_ = 0.0f, last_err_ = 0.0f, err_sum_ = 0.0f;
    float last_dout_ = 0.0f;
};


#endif //ROBOMASTER27ECTRAINING4TH_PID_HPP
