/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "Chassis.h"

void Chassis::ChassisSetVelocity(float _fbV, float _lrV, float _rtV) {
	FBVelocity = _fbV;
	LRVelocity = _lrV;
	RTVelocity = _rtV;
}

void Chassis::ChassisStop() {
	ChassisStopFlag = true;
}

void Chassis::ChassisActive() {
	ChassisStopFlag = false;
}

void Chassis::UpdataImuYaw(float _yaw)
{
	yaw = _yaw;
	chassisPos[2][0] = _yaw;
}


void Chassis::LSOdometry() {
	struct WheelSet {
		WheelSet(const float _angle, const float _vel) {
			angle = _angle / 180.f * PI; //弧度制
			float tmp[2];
			tmp[1] = _vel / 360.f * PI * WHEEL_DIAMETER * cosf(angle); //单位m/s
			tmp[0] = _vel / 360.f * PI * WHEEL_DIAMETER * sinf(angle);
			vel = tmp;
		}

		float angle{0};
		Matrixf<2, 1> vel;
	};

	WheelSet FR(SFR.GetState().position, -CFR.GetState().speed); //右侧轮毂电机反转，取负
	WheelSet FL(SFL.GetState().position, CFL.GetState().speed);
	WheelSet BL(SBL.GetState().position, CBL.GetState().speed);
	WheelSet BR(SBR.GetState().position, -CBR.GetState().speed);

	chassisVel = invA * (H1.trans() * FR.vel + H2.trans() * FL.vel + H3.trans() *
		BL.vel + H4.trans() * BR.vel);
	float WCSVeldata[3 * 1] = {
		chassisVel[0][0] * cosf(chassisPos[0][2]) - chassisVel[1][0] * sinf(chassisPos[0][2]),
		chassisVel[0][0] * sinf(chassisPos[0][2]) + chassisVel[1][0] * cosf(chassisPos[0][2]),
		chassisVel[0][2]
	};
	WCSVelocity = Matrixf<3, 1>(WCSVeldata);
	chassisPos += WCSVelocity * 0.001;

	//暂时用IMUyaw直接顶替
	chassisPos[2][0] = yaw;

	FRX = FR.vel[0][0], FRY = FR.vel[1][0];
	FLX = FL.vel[0][0], FLY = FL.vel[1][0];
	BLX = BL.vel[0][0], BLY = BL.vel[1][0];
	BRX = BR.vel[0][0], BRY = BR.vel[1][0];

	x = chassisPos[0][0];
	y = chassisPos[1][0];
	yaw = chassisPos[2][0];
}

void Chassis::ICFOdometry() {
	struct WheelSet {
		WheelSet(const float _angle, const float _vel) {
			angle = _angle / 180.f * PI; //弧度制
			float tmp[2];
			tmp[1] = _vel / 360.f * PI * WHEEL_DIAMETER * cosf(angle); //单位m/s
			tmp[0] = _vel / 360.f * PI * WHEEL_DIAMETER * sinf(angle);
			vel = tmp;
		}

		float angle{0};
		Matrixf<2, 1> vel;
	};

	WheelSet FR(SFR.GetState().position, -CFR.GetState().speed); //右侧轮毂电机反转，取负
	WheelSet FL(SFL.GetState().position, CFL.GetState().speed);
	WheelSet BL(SBL.GetState().position, CBL.GetState().speed);
	WheelSet BR(SBR.GetState().position, -CBR.GetState().speed);

	FRX = FR.vel[0][0], FRY = FR.vel[1][0];
	FLX = FL.vel[0][0], FLY = FL.vel[1][0];
	BLX = BL.vel[0][0], BLY = BL.vel[1][0];
	BRX = BR.vel[0][0], BRY = BR.vel[1][0];

	float plannedVel[3]{LRVelocity, FBVelocity, RTVelocity};
	x1 = alpha * x1 + (1-alpha) * Matrixf<3, 1>(plannedVel);
	x2 = alpha * x2 + (1-alpha) * Matrixf<3, 1>(plannedVel);
	x3 = alpha * x3 + (1-alpha) * Matrixf<3, 1>(plannedVel);
	x4 = alpha * x4 + (1-alpha) * Matrixf<3, 1>(plannedVel);

	J1 = matrixf::inv(alpha * alpha * matrixf::inv(J1) + 0.5 * Q);
	J2 = matrixf::inv(alpha * alpha * matrixf::inv(J2) + 0.5 * Q);
	J3 = matrixf::inv(alpha * alpha * matrixf::inv(J3) + 0.5 * Q);
	J4 = matrixf::inv(alpha * alpha * matrixf::inv(J4) + 0.5 * Q);

	Matrixf<3, 3> V1 = 0.25 * J1 + H1.trans() * B * H1;
	Matrixf<3, 1> v1 = 0.25 * J1 * x1 + H1.trans() * B * FR.vel;
	Matrixf<3, 3> V2 = 0.25 * J2 + H2.trans() * B * H2;
	Matrixf<3, 1> v2 = 0.25 * J2 * x2 + H2.trans() * B * FL.vel;
	Matrixf<3, 3> V3 = 0.25 * J3 + H3.trans() * B * H3;
	Matrixf<3, 1> v3 = 0.25 * J3 * x3 + H3.trans() * B * BL.vel;
	Matrixf<3, 3> V4 = 0.25 * J4 + H4.trans() * B * H4;
	Matrixf<3, 1> v4 = 0.25 * J4 * x4 + H4.trans() * B * BR.vel;

	Matrixf<3, 3> V1_t = 0.25 * (V1 + V2 + V3 + V4);
	Matrixf<3, 1> v1_t = 0.25 * (v1 + v2 + v3 + v4);
	// Matrixf<3, 3> V2_t = 0.25 * (V1 + V2 + V3 + V4);
	// Matrixf<3, 1> v2_t = 0.25 * (v1 + v2 + v3 + v4);
	// Matrixf<3, 3> V3_t = 0.25 * (V1 + V2 + V3 + V4);
	// Matrixf<3, 1> v3_t = 0.25 * (v1 + v2 + v3 + v4);
	// Matrixf<3, 3> V4_t = 0.25 * (V1 + V2 + V3 + V4);
	// Matrixf<3, 1> v4_t = 0.25 * (v1 + v2 + v3 + v4);

	V1 = V1_t, v1 = v1_t;
	V2 = V1_t, v2 = v1_t;
	V3 = V1_t, v3 = v1_t;
	V4 = V1_t, v4 = v1_t;

	x1 = matrixf::inv(V1) * v1;
	J1 = 4 * V1;
	x2 = matrixf::inv(V2) * v2;
	J2 = 4 * V2;
	x3 = matrixf::inv(V3) * v3;
	J3 = 4 * V3;
	x4 = matrixf::inv(V4) * v4;
	J4 = 4 * V4;


	float WCSVeldata[3 * 1] = {
		x1[0][0] * cosf(chassisPos[0][2]) - x1[1][0] * sinf(chassisPos[0][2]),
		x1[0][0] * sinf(chassisPos[0][2]) + x1[1][0] * cosf(chassisPos[0][2]),
		x1[0][2]
	};
	WCSVelocity = Matrixf<3, 1>(WCSVeldata);
	chassisPos += WCSVelocity * 0.001;

	//暂时用IMUyaw直接顶替
	chassisPos[2][0] = yaw;

	x = chassisPos[0][0];
	y = chassisPos[1][0];
	yaw = chassisPos[2][0];
}


void Chassis::WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity) {
	if(ChassisStopFlag){
		//设置底盘电机角度
		SFR.SetTargetAngle(45);
		SFL.SetTargetAngle(-45);
		SBL.SetTargetAngle(45);
		SBR.SetTargetAngle(-45);

		//设置底盘电机转速
		CFR.SetTargetSpeed(0);
		CFL.SetTargetSpeed(0);
		CBL.SetTargetSpeed(0);
		CBR.SetTargetSpeed(0);

		return;
	}
	float ChassisSpeed[4];
	static float lastAngle[4]{};

	const float A = lrVelocity - rtVelocity * LENGTH / 2;
	const float B = lrVelocity + rtVelocity * LENGTH / 2;
	const float C = fbVelocity - rtVelocity * WIDTH / 2;
	const float D = fbVelocity + rtVelocity * WIDTH / 2;

	//计算四个轮子角度，单位：度
	float RFRAngle = atan2f(A, D) * 180 / PI;
	float RFLAngle = atan2f(A, C) * 180 / PI;
	float RBLAngle = atan2f(B, C) * 180 / PI;
	float RBRAngle = atan2f(B, D) * 180 / PI;

	//计算四个轮子线速度，单位：度/s
	ChassisSpeed[0] = -sqrtf(A * A + D * D) / (WHEEL_DIAMETER * PI) * 360; //右前轮1
	ChassisSpeed[1] = sqrtf(A * A + C * C) / (WHEEL_DIAMETER * PI) * 360; //左前轮2
	ChassisSpeed[2] = sqrtf(B * B + C * C) / (WHEEL_DIAMETER * PI) * 360; //左后轮3
	ChassisSpeed[3] = -sqrtf(B * B + D * D) / (WHEEL_DIAMETER * PI) * 360; //右后轮4

	float motorAngleState[4];
	motorAngleState[0] = fmodf(SFR.GetState().position,360);
	motorAngleState[0] += motorAngleState[0]>180?-360:0;
	motorAngleState[1] = fmodf(SFL.GetState().position,360);
	motorAngleState[1] += motorAngleState[1]>180?-360:0;
	motorAngleState[2] = fmodf(SBL.GetState().position,360);
	motorAngleState[2] += motorAngleState[2]>180?-360:0;
	motorAngleState[3] = fmodf(SBR.GetState().position,360);
	motorAngleState[3] += motorAngleState[3]>180?-360:0;

	float FRerror = fabsf(motorAngleState[0]-RFRAngle);
	float FLerror = fabsf(motorAngleState[1]-RFLAngle);
	float BLerror = fabsf(motorAngleState[2]-RBLAngle);
	float BRerror = fabsf(motorAngleState[3]-RBRAngle);


	if(FRerror < 275 && FRerror>85){
		ChassisSpeed[0]*=-1;
		RFRAngle+=RFRAngle>0?-180:180;
	}
	if(FLerror < 275 && FLerror>85){
		ChassisSpeed[1]*=-1;
		RFLAngle+=RFLAngle>0?-180:180;
	}
	if(BLerror < 275 && BLerror>85){
		ChassisSpeed[2]*=-1;
		RBLAngle+=RBLAngle>0?-180:180;
	}
	if(BRerror < 275 && BRerror>85){
		ChassisSpeed[3]*=-1;
		RBRAngle+=RBRAngle>0?-180:180;
	}


	if (fabsf(fbVelocity) < 0.0001 && fabsf(lrVelocity) < 0.0001 && fabsf(rtVelocity) < 0.0001)
	{
		ChassisSpeed[0] = 0;
		ChassisSpeed[1] = 0;
		ChassisSpeed[2] = 0;
		ChassisSpeed[3] = 0;
		RFRAngle = lastAngle[0];
		RFLAngle = lastAngle[1];
		RBLAngle = lastAngle[2];
		RBRAngle = lastAngle[3];
	}
	// else if (FRerror > 5 || fabsf(FRerror - 180) > 5 || FLerror > 5 || fabsf(FLerror - 180) > 5 || BLerror > 5 ||
	// 	fabsf(BLerror - 180) > 5 || BRerror > 5 || fabsf(BRerror - 180) > 5)
	// {
	// 	ChassisSpeed[0] = 0;
	// 	ChassisSpeed[1] = 0;
	// 	ChassisSpeed[2] = 0;
	// 	ChassisSpeed[3] = 0;
	// }

	//设置底盘电机角度
	SFR.SetTargetAngle(RFRAngle);
	SFL.SetTargetAngle(RFLAngle);
	SBL.SetTargetAngle(RBLAngle);
	SBR.SetTargetAngle(RBRAngle);

	//设置底盘电机转速
	CFR.SetTargetSpeed(ChassisSpeed[0]);
	CFL.SetTargetSpeed(ChassisSpeed[1]);
	CBL.SetTargetSpeed(ChassisSpeed[2]);
	CBR.SetTargetSpeed(ChassisSpeed[3]);

	lastAngle[0]=RFRAngle;
	lastAngle[1]=RFLAngle;
	lastAngle[2]=RBLAngle;
	lastAngle[3]=RBRAngle;
}


void Chassis::Handle() {
	WheelsSpeedCalc(FBVelocity, LRVelocity, RTVelocity);
	// LSOdometry();
	ICFOdometry();
}


ChassisBuilder Chassis::Build() {
	return {};
}

void Chassis::ResetOdometry(float _x = 0, float _y = 0, float _angle = 0) {
	// chassisPos = matrixf::zeros<3, 1>();
	chassisPos[0][0] = _x;
	x = _x;
	chassisPos[1][0] = _y;
	y = _y;
	chassisPos[2][0] = _angle;
	yaw = _angle;
}

void Chassis::OffsetOdometry(float _x = 0, float _y = 0, float _angle = 0) {
	// chassisPos = matrixf::zeros<3, 1>();
	// chassisPos[0][0] += _x;
	// x += _x;
	// chassisPos[1][0] += _y;
	// y += _y;
	// chassisPos[2][0] += _angle;
	// yaw += _angle;
	chassisPos[2][0] += _angle;
	chassisPos[0][0] += _y * -sinf(chassisPos[2][0]) + _x * cosf(chassisPos[2][0]);
	chassisPos[1][0] += _y * cosf(chassisPos[2][0]) + _x * sinf(chassisPos[2][0]);
	x = chassisPos[0][0];
	y = chassisPos[1][0];
	yaw = chassisPos[2][0];
}








