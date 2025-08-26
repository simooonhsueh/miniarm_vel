#include "DmotorMx.h"

DmotorMx::DmotorMx()
	: Dmotor(4000000, 0, 64, 104, 108, 112, 116, 126, 128, 132, 11, 102, 122, 1, 4, 4, 4, 4, 2, 4, 4, 2, 1) {}

DmotorMx::DmotorMx(const unsigned char &MotorID, const string &MotorModel)
	: Dmotor(4000000, MotorID, 64, 104, 108, 112, 116, 126, 128, 132, 11, 102, 122, 1, 4, 4, 4, 4, 2, 4, 4, 2, 1)
{
	if (MotorModel == "Mx106" || MotorModel == "Mx64")
	{
		Motor_CenterScale = 2048;
		Max_Position_Limit = 4095;
		Min_Position_Limit = 0;
		Max_Velocity_Limit = 210;
		Min_Velocity_Limit = -210;
		Max_Accel_Limit = 32767;
		Max_Torque_Limit = 2047;
		Max_Extend_Limit = 1048575;
		Min_Extend_Limit = -1048575;
		Max_Value_In_1_rev = 4095;
		Min_Value_In_1_rev = 0;
		
		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Rev_Resol_Extended = Max_Value_In_1_rev - Min_Value_In_1_rev;
		Scale2RPM = 0.229;
		Scale2RPMM = 214.577;
	}
}