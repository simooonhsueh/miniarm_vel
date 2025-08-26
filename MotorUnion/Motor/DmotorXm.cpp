#include "DmotorXm.h"

DmotorXm::DmotorXm()
	: Dmotor(57600, 0, 64, 104, 108, 112, 116, 126, 128, 132, 11, 102, 122, 1, 4, 4, 4, 4, 2, 4, 4, 2, 1) {}

DmotorXm::DmotorXm(const unsigned char &MotorID, const string &MotorModel)
	: Dmotor(4000000, MotorID, 64, 104, 108, 112, 116, 126, 128, 132, 11, 102, 122, 1, 4, 4, 4, 4, 2, 4, 4, 2, 1)
{
	if (MotorModel == "Xm540" || MotorModel == "Xm430")
	{
		Motor_CenterScale = 2048;
		Max_Position_Limit = 4095;
		Min_Position_Limit = 0;
		Max_Velocity_Limit = 200;
		Min_Velocity_Limit = -200;
		Max_Accel_Limit = 32767;
		Max_Torque_Limit = 2047;

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Scale2RPM = 0.229;
		Scale2RPMM = 214.577;
	}
}