#include "motor.h"

motor::motor()
{
	Angle2MotorScale = 0.0f;
	MotorScale2Angle = 0.0f;
	Scale2RPM = 0.0f;
	Scale2RPMM = 0.0f;
	//default mode is position control mode
	Motor_Operating_Mode = POSITION;
	Motor_CenterScale = 0;
	Motor_Angle = 0.0f;
	Motor_Scale = 0;
	Motor_Velocity = 0;
	Motor_Profile_Velocity = 0;
	Motor_Accel = 0;
	Motor_Current = 0;
	Motor_Present_Angle = 0.0f;
	Motor_Present_Velocity = 0.0f;
	Motor_Present_Torque = 0.0f;
	Motor_TorqueEnable = false;
	Motor_PresentTorqueEnable = false;
	Motor_MovingState = false;

	Max_Position_Limit = 0;
	Min_Position_Limit = 0;
	Max_Velocity_Limit = 0;
	Min_Velocity_Limit = 0;
	Max_Extend_Limit = 0;
	Min_Extend_Limit = 0;
	Max_Accel_Limit = 0;
	Max_Torque_Limit = 0;

	is_Arrival = true;
	is_Write_Motor_Mode = false;
	is_Write_Scale = false;
	is_Write_Velocity = false;
	is_Write_Profile_Velocity = false;
	is_Write_Accel = false;
	is_Write_TorqueEnable = false;
	is_Write_Current = false;
}
//-----------------------------------------------------//
const unsigned char &motor::GetMotor_Operating_Mode() const { return Motor_Operating_Mode; }

const float &motor::GetMotor_Scale2RPM() const { return Scale2RPM; }

const float &motor::GetMotor_Scale2RPMM() const { return Scale2RPMM; }

const short &motor::GetMotor_CenterScale() const { return Motor_CenterScale; }

const float &motor::GetMotor_Angle() const { return Motor_Angle; }

const int &motor::GetMotor_Velocity() const { return Motor_Velocity; }

const int &motor::GetMotor_Accel() const { return Motor_Accel; }

const int &motor::GetMotor_Current() const { return Motor_Current; }

const bool &motor::GetMotor_TorqueEnable() const { return Motor_TorqueEnable; }

const float &motor::GetMotor_PresentAngle() const { return Motor_Present_Angle; }

const float &motor::GetMotor_PresentVelocity() const { return Motor_Present_Velocity; }

const float &motor::GetMotor_PresentTorque() const { return Motor_Present_Torque; }

const bool &motor::GetMotor_PresentTorqueEnable() const { return Motor_PresentTorqueEnable; }

const bool &motor::GetMotor_Motor_MovingState() const { return Motor_MovingState; }

const bool &motor::GetMotor_Arrival() const { return is_Arrival; }
//-----------------------------------------------------//
void motor::SetMotor_Operating_Mode(const unsigned char &operating_mode)
{
	Motor_Operating_Mode = static_cast<MOTOROPERATINGMODE>(operating_mode);
}

void motor::SetMotor_CenterScale(const short &centerscale)
{
	Motor_CenterScale = centerscale;
}

/**
 * @brief Set motor angle
 * 
 * @param angle - goal angle, in degree
 */
void motor::SetMotor_Angle(const float &angle)
{
	switch (Motor_Operating_Mode) {
	case CURRENTPOSITION:
	case VELOCITY:
	case POSITION:
		Motor_Scale = round(angle * Angle2MotorScale + Motor_CenterScale);
		if (Motor_Scale >= Max_Position_Limit)
			Motor_Scale = Max_Position_Limit;
		else if (Motor_Scale <= Min_Position_Limit)
			Motor_Scale = Min_Position_Limit;
		else
			;
		Motor_Angle = (Motor_Scale - Motor_CenterScale) * MotorScale2Angle;
		break;
	case EXPOSITION:
		Motor_Scale = round(angle / 360 * Rev_Resol_Extended) ;
		if (Motor_Scale >= Max_Extend_Limit)
			Motor_Scale = Max_Extend_Limit;
		else if (Motor_Scale <= Min_Extend_Limit)
			Motor_Scale = Min_Extend_Limit;
		else
			;
		Motor_Angle = (float)Motor_Scale / Rev_Resol_Extended * 360;
		
		break;
	default:
		break;
	}
	
	is_Arrival = false;
	is_Write_Scale = true;
}

/**
 * @brief Set motor velocity
 * 
 * @param velocity - goal velocity, in motor scale(RPM * unit)
 */
void motor::SetMotor_Velocity(const int &velocity)
{
	switch (Motor_Operating_Mode)
	{
	case VELOCITY:
		if (velocity >= Max_Velocity_Limit)
			Motor_Velocity = Max_Velocity_Limit;
		else if (velocity <= Min_Velocity_Limit)
			Motor_Velocity = Min_Velocity_Limit;
		else
			Motor_Velocity = velocity;
		break;
	case CURRENTPOSITION:
	case POSITION:
	case EXPOSITION:
		if (std::abs(velocity) >= Max_Velocity_Limit)
			Motor_Velocity = Max_Velocity_Limit;
		else
			Motor_Velocity = std::abs(velocity);
		break;
	default:
		break;
	}
	is_Write_Velocity = true;
}

void motor::SetMotor_Profile_Velocity(const int &velocity)
{
	switch (Motor_Operating_Mode)
	{
	case VELOCITY:
		if (velocity >= Max_Velocity_Limit)
			Motor_Profile_Velocity = Max_Velocity_Limit;
		else if (velocity <= Min_Velocity_Limit)
			Motor_Profile_Velocity = Min_Velocity_Limit;
		else
			Motor_Profile_Velocity = velocity;
		break;
	case CURRENTPOSITION:
	case POSITION:
	case EXPOSITION:
		if (std::abs(velocity) >= Max_Velocity_Limit)
			Motor_Profile_Velocity = Max_Velocity_Limit;
		else
			Motor_Profile_Velocity = std::abs(velocity);
		break;
	default:
		break;
	}
	is_Write_Profile_Velocity = true;
}

void motor::SetMotor_Accel(const int &accel)
{
	if (accel >= Max_Accel_Limit)
		Motor_Accel = Max_Accel_Limit;
	else
		Motor_Accel = accel;

	is_Write_Accel = true;
}

void motor::SetMotor_TorqueEnable(const bool &enable)
{
	Motor_TorqueEnable = enable;
	is_Write_TorqueEnable = true;
}

void motor::SetMotor_Current(const int &current)
{
	if (current >= Max_Torque_Limit)
		Motor_Current = Max_Torque_Limit;
	else if (current <= -Max_Torque_Limit)
		Motor_Current = -Max_Torque_Limit;
	else
		Motor_Current = current;

	is_Write_Current = true;
}

void motor::StopMotor()
{
	switch(Motor_Operating_Mode) {
		case VELOCITY:
			SetMotor_Velocity(0);
			SetMotor_TorqueEnable(false);
			break;
		case CURRENTPOSITION:
		case POSITION:
			SetMotor_Angle(Motor_Present_Angle);
			break;
		case EXPOSITION:
			SetMotor_Angle(Motor_Present_Angle);
			break;
		default:
			break;
	}
}