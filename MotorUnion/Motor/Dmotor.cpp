#include "Dmotor.h"

Dmotor::Dmotor()
	: motor(),
	  BAUDRATE(0),
	  Motor_ID(0),
	  ADDR_TORQUE_ENABLE(0),
	  ADDR_GOAL_VELOCITY(0),
	  ADDR_PROFILE_ACCEL(0),
	  ADDR_PROFILE_VELOCITY(0),
	  ADDR_GOAL_POSITION(0),
	  ADDR_PRESENT_TORQUE(0),
	  ADDR_PRESENT_VELOCITY(0),
	  ADDR_PRESENT_POSITION(0),
	  ADDR_OPERATING_MODE(0),
	  ADDR_GOAL_CURRENT(0),
	  ADDR_MOVING_STATE(0),
	  LEN_TORQUE_ENABLE(0),
	  LEN_GOAL_VELOCITY(0),
	  LEN_PROFILE_ACCEL(0),
	  LEN_PROFILE_VELOCITY(0),
	  LEN_GOAL_POSITION(0),
	  LEN_PRESENT_TORQUE(0),
	  LEN_PRESENT_VELOCITY(0),
	  LEN_PRESENT_POSITION(0),
	  LEN_GOAL_CURRENT(0),
	  LEN_MOVING_STATE(0)
{
	connected = false;
	read_count = 0;
}

Dmotor::Dmotor(
	const unsigned int &baudrate,
	const unsigned char &id,
	const uint16_t &addr_torque_enable,
	const uint16_t &addr_goal_velocity,
	const uint16_t &addr_profile_accel,
	const uint16_t &addr_profile_velocity,
	const uint16_t &addr_goal_position,
	const uint16_t &addr_present_torque,
	const uint16_t &addr_present_velocity,
	const uint16_t &addr_present_position,
	const uint16_t &addr_operating_mode,
	const uint16_t &addr_goal_current,
	const uint16_t &addr_moving_state,
	const uint16_t &len_torque_enable,
	const uint16_t &len_goal_velocity,
	const uint16_t &len_profile_accel,
	const uint16_t &len_profile_velocity,
	const uint16_t &len_goal_position,
	const uint16_t &len_present_torque,
	const uint16_t &len_present_velocity,
	const uint16_t &len_present_position,
	const uint16_t &len_goal_current,
	const uint16_t &len_moving_state)
	: motor(),
	  BAUDRATE(baudrate),
	  Motor_ID(id),
	  ADDR_TORQUE_ENABLE(addr_torque_enable),
	  ADDR_GOAL_VELOCITY(addr_goal_velocity),
	  ADDR_PROFILE_ACCEL(addr_profile_accel),
	  ADDR_PROFILE_VELOCITY(addr_profile_velocity),
	  ADDR_GOAL_POSITION(addr_goal_position),
	  ADDR_PRESENT_TORQUE(addr_present_torque),
	  ADDR_PRESENT_VELOCITY(addr_present_velocity),
	  ADDR_PRESENT_POSITION(addr_present_position),
	  ADDR_OPERATING_MODE(addr_operating_mode),
	  ADDR_GOAL_CURRENT(addr_goal_current),
	  ADDR_MOVING_STATE(addr_moving_state),
	  LEN_TORQUE_ENABLE(len_torque_enable),
	  LEN_GOAL_VELOCITY(len_goal_velocity),
	  LEN_PROFILE_ACCEL(len_profile_accel),
	  LEN_PROFILE_VELOCITY(len_profile_velocity),
	  LEN_GOAL_POSITION(len_goal_position),
	  LEN_PRESENT_TORQUE(len_present_torque),
	  LEN_PRESENT_VELOCITY(len_present_velocity),
	  LEN_PRESENT_POSITION(len_present_position),
	  LEN_GOAL_CURRENT(len_goal_current),
	  LEN_MOVING_STATE(len_moving_state)
{
	connected = false;
	read_count = 0;
}

const unsigned char &Dmotor::GetMotorID() const { return Motor_ID; }
const bool &Dmotor::GetMotorConnected() const { return connected; }

void Dmotor::ConnectDynamixel(
	dynamixel::PortHandler *portHandler,
	dynamixel::PacketHandler *packetHandler,
	dynamixel::GroupBulkRead *groupBulkRead,
	dynamixel::GroupBulkWrite *groupBulkWrite)
{
	this->portHandler = portHandler;
	this->packetHandler = packetHandler;
	this->groupBulkRead = groupBulkRead;
	this->groupBulkWrite = groupBulkWrite;
	ConnectDynamixel();
}

void Dmotor::ConnectDynamixel()
{
	uint8_t dxl_error = 0;
	uint16_t dxl_model_number = 0; // Dynamixel model number
	if (portHandler->openPort() & portHandler->setBaudRate(BAUDRATE))
	{
		int dxl_comm_result = packetHandler->ping(portHandler, Motor_ID, &dxl_model_number, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			// printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			portHandler->closePort();
			connected = false;
		}
		else if (dxl_error != 0)
		{
			// printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			portHandler->closePort();
			connected = false;
		}
		else
		{
			// printf("[ID:%03d] ping Succeeded. Dynamixel model number : %d\n", Motor_ID, dxl_model_number);
			connected = true;
		}
	}
	else
	{
		portHandler->closePort();
		connected = false;
	}
}

bool Dmotor::WriteData()
{
	// !!!!!! Must write torque enable first
	if (is_Write_TorqueEnable)
	{
		WriteTorqueEnable();
		return !is_Write_TorqueEnable;
	}
	if(is_Write_Motor_Mode)
	{
		WriteMode(Motor_Operating_Mode);
		return !is_Write_Motor_Mode;
	}
	if (is_Write_Accel)
	{
		WriteAccel();
		return !is_Write_Accel;
	}
	if (is_Write_Velocity)
	{
		WriteVelocity();
		return !is_Write_Velocity;
	}
	if (is_Write_Profile_Velocity)
	{
		WriteProfileVelocity();
		return !is_Write_Profile_Velocity;
	}
	if (is_Write_Scale)
	{
		WriteScale();
		return !is_Write_Scale;
	}
	if (is_Write_Current)
	{
		WriteCurrent();
		return !is_Write_Current;
	}
	return false;
}

void Dmotor::AddParam()
{
	switch (read_count)
	{
	case 0:
		AddParamPresentAngle();
		break;
	case 1:
		AddParamPresentVelocity();
		break;
	case 2:
		AddParamPresentTorque();
		break;
	case 3:
		AddParamPresentTorqueEnable();
		break;
	case 4:
		AddParamMovingState();
		break;
	}
}

void Dmotor::ReadData()
{
	switch (read_count)
	{
	case 0:
		ReadPresentAngle();
		if (abs(Motor_Angle - Motor_Present_Angle) < 1.0 || Motor_Operating_Mode == VELOCITY)	
			is_Arrival = true;
		else if (Motor_Operating_Mode == EXPOSITION && abs(Motor_Angle - Motor_Present_Angle) < 2.0f)
			is_Arrival = true;
		else
			is_Arrival = false;
		read_count = 1;
		break;
	case 1:
		ReadPresentVelocity();
		read_count = 2;
		break;
	case 2:
		ReadPresentTorque();
		read_count = 3;
		break;
	case 3:
		ReadPresentTorqueEnable();
		read_count = 4;
		break;
	case 4:
		ReadMovingState();
		read_count = 0;
		break;
	}
}

void Dmotor::AddParamPresentAngle()
{
	groupBulkRead->addParam(Motor_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
}

void Dmotor::AddParamPresentVelocity()
{
	groupBulkRead->addParam(Motor_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
}

void Dmotor::AddParamPresentTorque()
{
	groupBulkRead->addParam(Motor_ID, ADDR_PRESENT_TORQUE, LEN_PRESENT_TORQUE);
}

void Dmotor::AddParamPresentTorqueEnable()
{
	groupBulkRead->addParam(Motor_ID, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE);
}

void Dmotor::AddParamMovingState()
{
	groupBulkRead->addParam(Motor_ID, ADDR_MOVING_STATE, LEN_MOVING_STATE);
}

void Dmotor::ReadPresentAngle()
{
	int32_t data = groupBulkRead->getData(Motor_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
	if (Motor_Operating_Mode == EXPOSITION) Motor_Present_Angle = (data) * 0.088f;
	else Motor_Present_Angle = (data - Motor_CenterScale) * MotorScale2Angle;
}

void Dmotor::ReadPresentVelocity()
{
	int32_t data = groupBulkRead->getData(Motor_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
	Motor_Present_Velocity = data;
}

void Dmotor::ReadPresentTorque()
{
	int16_t data = groupBulkRead->getData(Motor_ID, ADDR_PRESENT_TORQUE, LEN_PRESENT_TORQUE);
	Motor_Present_Torque = data * 100.f / Max_Torque_Limit;
}

void Dmotor::ReadPresentTorqueEnable()
{
	int32_t data = groupBulkRead->getData(Motor_ID, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE);
	Motor_PresentTorqueEnable = data;
}

void Dmotor::ReadMovingState()
{
	int32_t data = groupBulkRead->getData(Motor_ID, ADDR_MOVING_STATE, LEN_MOVING_STATE);
	Motor_MovingState = data;
}

void Dmotor::WriteMode(uint8_t mode)	// NOT WORKING!!!
{
	uint8_t param_mode[LEN_TORQUE_ENABLE];
	param_mode[0] = mode;
	groupBulkWrite->addParam(Motor_ID, ADDR_OPERATING_MODE, LEN_TORQUE_ENABLE, param_mode);
	is_Write_Motor_Mode = false;
}

void Dmotor::WriteScale()
{
	uint8_t param_goal_position[LEN_GOAL_POSITION];
	param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD((int)Motor_Scale));
	param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD((int)Motor_Scale));
	param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD((int)Motor_Scale));
	param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD((int)Motor_Scale));
	
	groupBulkWrite->addParam(Motor_ID, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_goal_position);
	is_Write_Scale = false;
}

void Dmotor::WriteVelocity()
{
	uint8_t param_goal_velocity[LEN_GOAL_VELOCITY];
	param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD((int)Motor_Velocity));
	param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD((int)Motor_Velocity));
	param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD((int)Motor_Velocity));
	param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD((int)Motor_Velocity));

	switch (Motor_Operating_Mode)
	{
	case VELOCITY:
		groupBulkWrite->addParam(Motor_ID, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, param_goal_velocity);
		break;

	case POSITION:
	case EXPOSITION:
		groupBulkWrite->addParam(Motor_ID, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, param_goal_velocity);
		groupBulkWrite->addParam(Motor_ID, ADDR_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY, param_goal_velocity);
		break;
	}

	is_Write_Velocity = false;
}

void Dmotor::WriteProfileVelocity()
{
	uint8_t param_profile_velocity[LEN_PROFILE_VELOCITY];
	param_profile_velocity[0] = DXL_LOBYTE(DXL_LOWORD((int)Motor_Profile_Velocity));
	param_profile_velocity[1] = DXL_HIBYTE(DXL_LOWORD((int)Motor_Profile_Velocity));
	param_profile_velocity[2] = DXL_LOBYTE(DXL_HIWORD((int)Motor_Profile_Velocity));
	param_profile_velocity[3] = DXL_HIBYTE(DXL_HIWORD((int)Motor_Profile_Velocity));

	switch (Motor_Operating_Mode)
	{
	case VELOCITY:
		groupBulkWrite->addParam(Motor_ID, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, param_profile_velocity);
		break;

	case POSITION:
	case EXPOSITION:
		//groupBulkWrite->addParam(Motor_ID, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, param_profile_velocity);
		groupBulkWrite->addParam(Motor_ID, ADDR_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY, param_profile_velocity);
		break;
	}

	is_Write_Profile_Velocity = false;
}


void Dmotor::WriteAccel()
{
	uint8_t param_goal_accel[LEN_PROFILE_ACCEL];
	param_goal_accel[0] = DXL_LOBYTE(DXL_LOWORD(Motor_Accel));
	param_goal_accel[1] = DXL_HIBYTE(DXL_LOWORD(Motor_Accel));
	param_goal_accel[2] = DXL_LOBYTE(DXL_HIWORD(Motor_Accel));
	param_goal_accel[3] = DXL_HIBYTE(DXL_HIWORD(Motor_Accel));

	groupBulkWrite->addParam(Motor_ID, ADDR_PROFILE_ACCEL, LEN_PROFILE_ACCEL, param_goal_accel);
	is_Write_Accel = false;
}

void Dmotor::WriteTorqueEnable()
{
	uint8_t param_torque_enable[LEN_TORQUE_ENABLE];
	param_torque_enable[0] = Motor_TorqueEnable;

	groupBulkWrite->addParam(Motor_ID, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE, param_torque_enable);
	is_Write_TorqueEnable = false;
}

void Dmotor::WriteCurrent()
{
	uint8_t param_goal_current[LEN_GOAL_CURRENT];
	param_goal_current[0] = DXL_LOBYTE(Motor_Current);
	param_goal_current[1] = DXL_HIBYTE(Motor_Current);

	groupBulkWrite->addParam(Motor_ID, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT, param_goal_current);
	is_Write_Current = false;
}
