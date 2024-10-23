/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MANIPULATOR_H
#define FINEMOTE_MANIPULATOR_H

#endif //FINEMOTE_MANIPULATOR_H


#include "ProjectConfig.h"
#include "DeviceBase.h"
#include "MotorBase.h"

class Manipulator : public DeviceBase {
public:
	Manipulator(MotorBase* _motorA, MotorBase* _motorB, MotorBase* _motorC, MotorBase* _motorD, MotorBase* _motorE,MotorBase* _motorF):
		motorA(_motorA), motorB(_motorB), motorC(_motorC), motorD(_motorD),motorE(_motorE), motorF(_motorF) {}
	void UpdataEncoderData(const float _CAngle,const float _DAngle,const float _EAngle) {
		EncoderCAngle=_CAngle;
		EncoderDAngle=_DAngle;
		EncoderCAngle=_EAngle;
	}
	void Handle() override {

	}
private:
	MotorBase* motorA;
	MotorBase* motorB;
	MotorBase* motorC;
	MotorBase* motorD;
	MotorBase* motorE;
	MotorBase* motorF;

	float EncoderCAngle{0};
	float EncoderDAngle{0};
	float EncoderEAngle{0};

	const float AAngleOffset{0};
	const float BAngleOffset{0};
	const float CAngleOffset{0};
	const float DAngleOffset{0};
	const float EAngleOffset{0};
	const float FAngleOffset{0};

};
