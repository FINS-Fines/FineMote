/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_PID_HPP
#define FINEMOTE_PID_HPP

#include "ImplementControlBase.hpp"

typedef struct PID_Param_t {
    float kp;
    float ki;
    float kd;

    // 积分上下限
    float iMax;

    // 输出上下限
    float outMax;

} PID_Param_t;

class PID : public ImplementControllerBase<1, 1> {
public:
    constexpr explicit PID(const PID_Param_t& params) : params(params) {}

    void PerformCalc() override {
        float error = *(this->targetPtrs[0]) - *(this->feedbackPtrs[0]);
        totalError += error;
        Clamp(totalError, -params.iMax, params.iMax);       // 积分项限幅

        float output = params.kp * error + params.ki * totalError + params.kd * (error - lastError);
        this->outputs[0] = Clamp(output, -params.outMax, params.outMax);  // 输出限幅

        lastError = error;
    }

    ControllerOutputData GetOutputs() override {
        return { this->outputs.data(), 1 };
    }

private:
    const PID_Param_t params;
    float totalError = 0;
    float lastError = 0;
};

template<size_t K>
class CascadePID : public PID {
public:
    template<typename... Params>
    constexpr explicit CascadePID(const PID_Param_t& first, Params... params) : PID(first), nodes{PID(params)...} {
        static_assert(K > 1, "CascadePID should have at least 2 layers");

        Cascade(nodes[0]);
        for (int i = 0; i < K - 2; ++i) {
            nodes[i].Cascade(nodes[i + 1]);
        }
    }

    void PerformCalc() final{
        PID::Calc();
        for(auto& node : nodes) {
            node.Calc();
        }

        this->outputs[0] = nodes.rbegin()->GetOutput();
    }

    void SetFeedback(const std::vector<float*>& feedbackPtrs) {
        auto iter = feedbackPtrs.begin();

        PID::SetFeedbacks(*iter);
        ++iter;

        for (auto& node : nodes) {
            node.SetFeedbacks(*iter);
            ++iter;
        }
    }


private:
    std::array<PID, K - 1> nodes;
};

#endif
