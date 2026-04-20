/*******************************************************************************
* Copyright (c) 2026.
* IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
* All rights reserved.
******************************************************************************/

#ifndef FINEMOTE_LQR_HPP
#define FINEMOTE_LQR_HPP
#include "ImplementControlBase.hpp"
#include <Dense> // 采用Eigen5.0.0代码

template <uint8_t StatusSize, uint8_t ControlSize>
// StatusSize表示系统状态的维数
// ControlSize表示LQR求出的系统输入的维数
struct LQR_Param_t
{
    using MatrixK = Eigen::Matrix<float, ControlSize, StatusSize>;
    // 反馈项系数K ControlSize * StatusSize
    MatrixK K = MatrixK::Zero();
    // 前馈项系数（可选）F ControlSize * StatusSize
    MatrixK F = MatrixK::Zero();
    // 构造函数，F参数可缺省
    constexpr explicit LQR_Param_t() = default;
    constexpr explicit LQR_Param_t(MatrixK K, MatrixK F = MatrixK::Zero) : K(K), F(F) {}
};

template <uint8_t StatusSize, uint8_t ControlSize>
// StatusSize表示系统状态的维数
// ControlSize表示LQR求出的系统输入的维数
struct System_Param_t
{
    // 状态矩阵 A: StatusSize x StatusSize
    using MatrixA = Eigen::Matrix<float, StatusSize, StatusSize>;
    MatrixA A = MatrixA::Zero();

    // 输入矩阵 B: StatusSize x ControlSize
    using MatrixB = Eigen::Matrix<float, StatusSize, ControlSize>;
    MatrixB B = MatrixB::Zero();

    // 权重矩阵 Q: StatusSize x StatusSize
    using MatrixQ = MatrixA;
    MatrixQ Q = MatrixQ::Zero();

    // 权重矩阵 R: ControlSize x ControlSize
    using MatrixR = Eigen::Matrix<float, ControlSize, ControlSize>;
    MatrixR R = MatrixR::Zero();

    constexpr explicit System_Param_t(MatrixA A, MatrixB B, MatrixQ Q, MatrixR R) : A(A), B(B), Q(Q), R(R) {}
};

template<uint8_t StatusSize, uint8_t ControlSize>
class LQR : public ImplementControllerBase<StatusSize, StatusSize, ControlSize> // 目标&反馈都与状态变量等阶
{
public:
    // 1. 用户直接提供离线求解的K矩阵和F矩阵
    constexpr explicit LQR(const LQR_Param_t<StatusSize, ControlSize>& LQR_Param) : param(LQR_Param) {}
    // 2. 用户提供系统的AB矩阵，以及代价函数的QR矩阵
    explicit LQR(const System_Param_t<StatusSize, ControlSize>& System_Param) : param(computeParam(System_Param)) {}
    void PerformCalc() override
    {
        // 计算target向量与error向量
        Eigen::Matrix<float, StatusSize, 1> target;
        Eigen::Matrix<float, StatusSize, 1> error;
        for(size_t i = 0; i < StatusSize; ++i){
            target(i) = *(this->targetPtrs[i]);
            error(i) = *(this->targetPtrs[i]) - *(this->feedbackPtrs[i]);
        }

        // 使用Eigen矩阵运算计算outputs的反馈项: u = K * error
        // 符号问题：因为error定义成了target-feedback，因此K不能带负号
        Eigen::Matrix<float, ControlSize, 1> feedback = param.K * error;

        // 接下来计算前馈项：u_ff = F * target
        Eigen::Matrix<float, ControlSize, 1> feedforward = param.F * target;

        // 将结果复制到outputs数组
        for(size_t i = 0; i < ControlSize; ++i){
            this->outputs[i] = feedback(i) + feedforward(i);
        }
    }
    ControllerOutputData GetCurrentOutputs() override {
        return { (float*)this->outputs.data(), (uint8_t)ControlSize };
    }

    ControllerOutputData GetTotalOutputs() override {
        return { (float*)this->outputs.data(), (uint8_t)ControlSize };
    }

    LQR_Param_t<StatusSize, ControlSize> getParam () const{
        return param;
    };
private:
    const LQR_Param_t<StatusSize, ControlSize> param;
    LQR_Param_t<StatusSize, ControlSize> computeParam(System_Param_t<StatusSize, ControlSize> System_Param)
    {
        LQR_Param_t<StatusSize, ControlSize> param;
        // 1.先判定ABQR输入是否合法
        // 1.1 检查维度是否合理
        static_assert(StatusSize > 0, "StatusSize must be greater than 0");
        static_assert(ControlSize > 0, "ControlSize must be greater than 0");

        // 1.2 检查Q和R的正定性质
        // Q必须为半正定，R必须为正定
        // Eigen::SelfAdjointEigenSolver<typename System_Param_t<StatusSize, ControlSize>::MatrixQ> qSolver(System_Param.Q);
        // Eigen::SelfAdjointEigenSolver<typename System_Param_t<StatusSize, ControlSize>::MatrixR> rSolver(System_Param.R);
        // for(int i = 0; i < StatusSize; ++i) {
        //     static_assert(qSolver.eigenvalues()(i) >= 0, "Q must be positive semidefinite matrix!");
        // }
        //
        // for(int i = 0; i < ControlSize; ++i) {
        //     static_assert(qSolver.eigenvalues()(i) > 0, "R must be positive definite matrix!");
        // }

        // 2.然后使用Eigen，求解离散时间代数Riccati方程
        constexpr int maxIterations = 1000; // 迭代次数
        constexpr float tolerance = 1e-6f; // 迭代精度

        using MatrixP = Eigen::Matrix<float, StatusSize, StatusSize>;
        MatrixP P = System_Param.Q; // 初始猜测
        MatrixP P_prev; // 上一次迭代的P矩阵

        for(int iter = 0; iter < maxIterations; ++iter) {
            P_prev = P;

            // 离散时间代数Riccati方程迭代
            // P = A'PA - A'PB(B'PB + R)^{-1}B'PA + Q
            Eigen::Matrix<float, ControlSize, ControlSize> temp = System_Param.B.transpose() * P * System_Param.B + System_Param.R;
            // static_assert(temp.determinant() != 0, "Matrix (B'PB + R) is not invertible!");
            Eigen::Matrix<float, ControlSize, ControlSize> tempInv = temp.inverse();
            P = System_Param.A.transpose() * P * System_Param.A -
                System_Param.A.transpose() * P * System_Param.B * tempInv * System_Param.B.transpose() * P * System_Param.A +
                System_Param.Q;

            // 检查收敛
            if((P - P_prev).norm() < tolerance) {
                break;
            }

            // 如果达到最大迭代次数仍未收敛，使用当前P计算K
        }
        // 3.给K矩阵赋值
        // 计算K矩阵: K = (B'PB + R)^{-1}B'PA
        Eigen::Matrix<float, ControlSize, ControlSize> finalTemp = System_Param.B.transpose() * P * System_Param.B + System_Param.R;
        // static_assert(finalTemp.determinant() != 0, "Matrix (B'PB + R) is not invertible!");
        param.K = finalTemp.inverse() * System_Param.B.transpose() * P * System_Param.A;

        // 4.给F矩阵赋值
        Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<float, StatusSize, ControlSize>> cod(System_Param.B);
        Eigen::Matrix<float, ControlSize, StatusSize> B_pinv = cod.pseudoInverse();
        // 计算 F = pinv(B) * (I - A)
        param.F = B_pinv * (Eigen::Matrix<float, StatusSize, StatusSize>::Identity() - System_Param.A);

        return param;
    }
};

#endif // FINEMOTE_LQR_HPP
