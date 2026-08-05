/*******************************************************************************
 * Copyright (c) 2026
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_HT_RS485_MOTOR_V2_HPP
#define FINEMOTE_HT_RS485_MOTOR_V2_HPP

#include "Bus/RS485_Base.hpp"
#include "MotorBase.hpp"
#include "Verification/CRC.hpp"

inline constexpr size_t MOTOR_MAP_LENGTH = 10;

template <uint8_t ID>
class HTMotorProxy_RS485 {
public:
    /**
     * @brief 构造并注册电机到指定 RS485 总线代理
     *
     * 初始化对应总线的 RS485 基类单例，并将给定 MotorBase 指针按地址插入内部电机映射表。
     * 这样当接收到来自该地址的数据时，代理能够在映射表中查找并将数据分发到对应的电机实例进行解码与状态更新。
     *
     * @tparam ID 模板参数，表示所属的 RS485 总线/代理实例标识
     * @param motor 指向要注册的 MotorBase 实例的指针；对象的生命周期由外部管理，该构造函数不会接管所有权
     * @param addr  在 RS485 总线上的设备地址（从机 ID），用于在内部映射表中作为键
     */
    /**
     * @brief 构造并注册电机实例到指定的 RS485 代理
     *
     * 将 motor 指针插入到代理内部的地址映射表中，并确保对应的 RS485 总线单例已初始化。
     * 参数 motor 的生命周期由外部管理，构造函数不负责释放。
     *
     * @param motor 要注册的 MotorBase 指针
     * @param addr  在 RS485 总线上的设备地址（从机 ID）
     */
    HTMotorProxy_RS485(MotorBase* motor, uint8_t addr) {
        RS485_Base<ID>::GetInstance();
        getMotorMap().insert(etl::make_pair(addr, motor));
    }

    /**
     * @brief 通过所属代理发送原始数据
     *
     * 将给定的字节缓冲区交由内部的 RS485_Agent 发送。
     *
     * @param data 指向要发送的数据缓冲区
     * @param size 缓冲区长度（字节）
     */
    static void Transmit(uint8_t* data, size_t size) {
        rs485Agent.Transmit(data, size);
    }

private:
    static RS485_Agent<ID> rs485Agent;

    /**
     * @brief 返回代理内部的地址 -> MotorBase* 映射表的单例引用
     *
     * 映射表用于在接收到报文时根据设备地址查找对应的电机实例并分发数据。
     *
     * @return etl::map<uint8_t, MotorBase*, MOTOR_MAP_LENGTH>& 映射表引用
     */
     static etl::map<uint8_t, MotorBase*, MOTOR_MAP_LENGTH>& getMotorMap() {
        static etl::map<uint8_t, MotorBase*, MOTOR_MAP_LENGTH> instance;
        return instance;
    }

    /**
     * @brief 解码从 RS485 总线接收的电机应答数据并更新对应 MotorBase 状态
     *
     * 本函数校验报文长度与 CRC，然后根据报文中的设备地址查找已注册的电机，
     * 并把接收到的位置/速度信息解码到电机状态中（单位转换与符号按协议处理）。
     * 未包含电流和温度字段时会将其置为 0。
     *
     * @param data 指向接收到的报文字节
     * @param size 报文长度（字节），期望为 15
     */
    static void Decode(uint8_t* data, size_t size) {
        auto& motorMap = getMotorMap();
        if (size != 15) return;
        CRC_t<crc16_modbus> crc;
        if (crc.Calc(data, 13u) != (data[13] | data[14] << 8u)) return;
        if (!motorMap.contains(data[2])) return;
        if (data[3] == 0x55) {
            MotorBase* motor = motorMap[data[2]];
            // Motor_State_t state;
            // state.position =
            //      -1 * (static_cast<float>(data[7] | (data[8] << 8u) | (data[9] << 16u) | (data[10] << 24u)) * 360.0f / 16384.0f);
            // state.speed = -1.0f * static_cast<float>(data[11] | (data[12] << 8u));
            // state.torque = 0; // 电机应答不返回电流值
            // state.temperature = 0; // 电机应答不返回温度参数
            motor->SetState(Motor_State_t{
                -1.0f * (static_cast<float>(data[7] | (data[8] << 8u) | (data[9] << 16u) | (data[10] << 24u)) * 360.0f / 16384.0f),
                -1.0f * static_cast<float>(data[11] | (data[12] << 8u)),
                0.0f,// 电机应答不返回电流值
            });
        }
    }
};

template <uint8_t ID>
RS485_Agent<ID> HTMotorProxy_RS485<ID>::rs485Agent(0x3C, [](uint8_t* data, size_t size) {
    Decode(data, size);
});


template<uint8_t BusID>
class HT_RS485_Motor_v2: public MotorBase {
public:
    /**
     * @brief 构造 HT_RS485_MOTOR_V2 实例并注册到指定总线代理
     *
     * 初始化基类 MotorBase，设置设备地址并创建对应的通信代理（将自身注册到代理映射），
     * 同时将外部提供的控制器与电机绑定。
     *
     * @tparam T 控制器类型
     * @param params 电机参数（右值引用）
     * @param _controller 控制器实例引用，用于驱动该电机
     * @param addr 在总线上的设备地址
     * @param divisionFactor 控制器分频因子，默认 5
     */
    template<typename T>
    HT_RS485_Motor_v2(const Motor_Param_t&& params, T& _controller, uint8_t addr, uint32_t divisionFactor = 5):
        MotorBase(std::forward<const Motor_Param_t>(params), divisionFactor),
        id(addr),temperature(0),
        commuAgent(this, addr) { // Todo: ID和地址分离逻辑
        ResetController(_controller);
    }

protected:
    /**
     * @brief 根据控制类型设置控制器的反馈源
     *
     * 当前仅在位置控制模式下将电机的 position 作为控制器的反馈量。
     */
    void SetFeedback() final {
        switch (params.ctrlType) {
            case Motor_Ctrl_Type_e::Position:
                controller->SetFeedbacks(&state.position);
                break;
            default:
                break;
        }
    }

    /**
     * @brief 周期性处理函数：计算控制器输出并生成发送报文
     *
     * 在任务循环中被调用以推进控制器状态并将控制命令发送到电机。
     */
    void Handle() final {
        controller->Calc();
        MessageGenerate();
    }

    /**
     * @brief 状态更新钩子，由上层调用以同步或刷新电机状态
     *
     * 当前实现为空，保留给未来需要在更新周期中执行的逻辑。
     */
    void Update() final {

    }

private:
    uint8_t txbuf[11] = {};

    /**
     * @brief 根据控制器输出构建并发送 RS485 控制报文
     *
     * 按电机协议格式填充发送缓冲区（包含头、ID、命令码、数据与 CRC），
     * 并通过通信代理发送出去。当前仅实现位置控制命令的构建。
     */
    void MessageGenerate() {
        switch (params.ctrlType) {
            case Motor_Ctrl_Type_e::Position: {
                ControllerOutputData output = controller->GetOutputs();
                float targetAngle = -1 * output.dataPtr[0];
                auto txAngle = static_cast<int32_t>(targetAngle * 16384.0f / 360.0f);

                txbuf[0] = 0x3E; // 协议头
                txbuf[1] = 0x00; // 包序号
                txbuf[2] = id; // ID
                txbuf[3] = 0x55; // 绝对位置闭环控制命令码
                txbuf[4] = 0x04; // 数据包长度
                txbuf[5] = txAngle;
                txbuf[6] = txAngle >> 8u;
                txbuf[7] = txAngle >> 16u;
                txbuf[8] = txAngle >> 24u;
                uint16_t crc16 = CRCCalc<crc16_modbus>(txbuf, 9u);
                txbuf[9] = crc16;
                txbuf[10] = crc16 >> 8u;
                commuAgent.Transmit(txbuf, 11);
                break;
            }
            default:break;;
        }
    }

    const uint8_t id;
    int8_t temperature; //电机温度，单位摄氏度
    HTMotorProxy_RS485<BusID> commuAgent;
};

#endif
