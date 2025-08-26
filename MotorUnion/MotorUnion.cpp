#include "MotorUnion.h"
#include <unistd.h>
#include <math.h>
vector<unsigned char> MotorUnion::allport = {0, 1, 2, 3, 4, 5, 6};

/**
 * Motor constructor
 * 
 * @param IDArray - 
 * @param MotorModelArray - Array defined the type of motor using
 */
MotorUnion::MotorUnion(const vector<unsigned char> &IDArray,
					   const vector<string> &MotorModelArray)
	: _is_deleted_thread_BG(true),
	  waiting_frequency(10)
{
	for (unsigned char i = 0; i < IDArray.size(); i++)
	{
		if (MotorModelArray.at(i) == "Pro100" || MotorModelArray.at(i) == "Pro200" || MotorModelArray.at(i) == "Pro20" || MotorModelArray.at(i) == "Pro10" || MotorModelArray.at(i) == "RH")
			Motor_Union.push_back(new DmotorPro(IDArray.at(i), MotorModelArray.at(i)));

		else if (MotorModelArray.at(i) == "Pro20+")
			Motor_Union.push_back(new DmotorProPlus(IDArray.at(i), MotorModelArray.at(i)));

		else if (MotorModelArray.at(i) == "Mx106" || MotorModelArray.at(i) == "Mx64")
			Motor_Union.push_back(new DmotorMx(IDArray.at(i), MotorModelArray.at(i)));

		else if (MotorModelArray.at(i) == "Xm540" || MotorModelArray.at(i) == "Xm430")
			Motor_Union.push_back(new DmotorXm(IDArray.at(i), MotorModelArray.at(i)));

		else
			;
	}

	if (ConnectAllMotors(MotorUnion::allport))
		BGON();
	else
		std::cout << "\033[1;31m[MotorUnion] Connection failed.\033[0m" << std::endl;
}

MotorUnion::~MotorUnion()
{
	if (!_is_deleted_thread_BG)
	{
		_is_deleted_thread_BG = true;
		thread_BG->join();
		delete thread_BG;
	}

	deleteInVector(Motor_Union);
	deleteInVector(portHandler);
	deleteInVector(groupBulkRead);
	deleteInVector(groupBulkWrite);
}

template <class T>
void MotorUnion::deleteInVector(vector<T *> tmp_vector)
{
	while (!tmp_vector.empty())
	{
		delete tmp_vector.back();
		tmp_vector.pop_back();
	}
}

////////////////////////////////////////////////////////////////////////////////
///  All Motors   //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
const bool MotorUnion::ConnectAllMotors(vector<unsigned char> &AllPortNumber)
{
	vector<unsigned char>::iterator it;
	for (it = AllPortNumber.begin(); it != AllPortNumber.end();)
	{
		// Set the port path and check if port exist
		string port_path = string("/dev/ttyUSB" + to_string(*it));
		if( access( port_path.c_str(), F_OK ) == -1 ) {
			it++;
			continue;
		}

		// Initialize PortHandler & PacketHandler instance
		dynamixel::PortHandler *tmp_portHandler = dynamixel::PortHandler::getPortHandler(port_path.c_str());
		packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
		dynamixel::GroupBulkRead *tmp_groupBulkRead = new dynamixel::GroupBulkRead(tmp_portHandler, packetHandler);
		dynamixel::GroupBulkWrite *tmp_groupBulkWrite = new dynamixel::GroupBulkWrite(tmp_portHandler, packetHandler);

		bool port_gate = false;
		for (int i = 0; i < Motor_Union.size(); i++)
		{
			if (Motor_Union.at(i)->GetMotorConnected() == false)
			{
				Motor_Union.at(i)->ConnectDynamixel(tmp_portHandler, packetHandler, tmp_groupBulkRead, tmp_groupBulkWrite);
				port_gate |= Motor_Union.at(i)->GetMotorConnected();
			}
		}
		if (port_gate == true)
		{
			it = AllPortNumber.erase(it);
			portHandler.push_back(tmp_portHandler);
			groupBulkRead.push_back(tmp_groupBulkRead);
			groupBulkWrite.push_back(tmp_groupBulkWrite);
		}
		else
		{
			++it;
			delete tmp_groupBulkWrite;
			delete tmp_groupBulkRead;
			delete tmp_portHandler;
		}
	}

	// Check every motor is connected
	bool connected = true;
	for (int i = 0; i < Motor_Union.size(); i++)
		connected &= Motor_Union.at(i)->GetMotorConnected();

	return connected;
}

const bool MotorUnion::CheckAllMotorsConnected() const
{
	bool connected = true;
	for (auto &motor : Motor_Union)
		connected &= motor->GetMotorConnected();
	return connected;
}

const bool MotorUnion::CheckAllMotorsArrival() const
{
	bool arrival = true;
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		if (!Motor_Union.at(i)->GetMotor_Arrival())
		printf("Motor %i with target %f/%f\n", i, Motor_Union.at(i)->Motor_Angle, Motor_Union.at(i)->Motor_Present_Angle);
		arrival &= Motor_Union.at(i)->GetMotor_Arrival();
	}
	return arrival;
}

/**
 * Wait for i-th motor to arrive
 * 
 * @param i - ID of the motor
 */
void MotorUnion::WaitMotorArrival(int i) const {
	while (!*terminated_ && !Motor_Union.at(i)->GetMotor_Arrival() && Motor_Union.at(i)->GetMotorConnected()) {
		if (GetAllMotorsTorqueEnable() == false)
			break;
		this_thread::sleep_for(chrono::milliseconds(waiting_frequency));
	}
}

void MotorUnion::WaitAllMotorsArrival() const
{	
	while (!*terminated_ && !CheckAllMotorsArrival() && CheckAllMotorsConnected())
	// while (!*terminated_ && !CheckAllMotorsArrival())
	{
		if (GetAllMotorsTorqueEnable() == false)
			break;
		this_thread::sleep_for(chrono::milliseconds(waiting_frequency));
	}
}

void MotorUnion::WaitAllMotorsArrival(const int &total_waiting_time_ms) const
{
	if (total_waiting_time_ms <= 0)
		return;
	for (int i = 0; i < total_waiting_time_ms / waiting_frequency; i++)
	{
		if (GetAllMotorsTorqueEnable() == false)
			break;
		this_thread::sleep_for(chrono::milliseconds(waiting_frequency));
	}
}

void MotorUnion::StopMotor(int idx) const {
	Motor_Union.at(idx)->StopMotor();
}

void MotorUnion::StopAllMotors() const {
	for (int i = 0; i < Motor_Union.size(); i++) {
		StopMotor(i);
	}
}

//------------------------------------------------------------------------------//
/*
	Get All Motors Data
*/

const size_t MotorUnion::GetMotor_Number() const
{
	return Motor_Union.size();
}

const bool MotorUnion::GetAllMotorsTorqueEnable() const
{
	bool tmp = true;
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		tmp &= GetMotor_TorqueEnable(i);
	}
	return tmp;
}


const bool MotorUnion::GetAnyMotorsTorqueEnable() const {
	bool tmp = true;
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		tmp |= GetMotor_TorqueEnable(i);
	}
	return tmp;
}

//------------------------------------------------------------------------------//
/*
	Set Motors Data
*/
void MotorUnion::SetAllMotorsOperatingMode(const unsigned char &mode) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Operating_Mode(i, mode);
	}
}

void MotorUnion::SetAllMotorsAngle(const float &angle) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Angle(i, angle);
	}
}

void MotorUnion::SetAllMotorsVelocity(const int &velocity) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Velocity(i, velocity);
	}
}

void MotorUnion::SetAllMotorsProfileVelocity(const int &velocity) const {
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Profile_Velocity(i, velocity);
	}
}

void MotorUnion::SetAllMotorsAccel(const int &accel) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Accel(i, accel);
	}
}

void MotorUnion::SetAllMotorsTorqueEnable(const bool &enable) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_TorqueEnable(i, enable);
	}
}

////////////////////////////////////////////////////////////////////////////////
///   Motor   //////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------//
/*
	Get Motor Data
*/
const unsigned char &MotorUnion::GetMotor_ID(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotorID();
}

const bool &MotorUnion::GetMotor_Connected(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotorConnected();
}

const bool MotorUnion::CheckPort_Connected() const
{
	return portHandler.empty();
}

void MotorUnion::SetTerminated_flag(bool *terminated) {
	terminated_ = terminated;
}

bool MotorUnion::GetMotor_Terminated(void) const {
	return *terminated_;
}

const float &MotorUnion::GetMotor_Scale2RPM(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Scale2RPM();
}

const float &MotorUnion::GetMotor_Scale2RPMM(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Scale2RPMM();
}

const short &MotorUnion::GetMotor_CenterScale(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_CenterScale();
}

const float &MotorUnion::GetMotor_Angle(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Angle();
}

const int &MotorUnion::GetMotor_Velocity(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Velocity();
}

const int &MotorUnion::GetMotor_Accel(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Accel();
}

const bool &MotorUnion::GetMotor_TorqueEnable(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_TorqueEnable();
}

const float &MotorUnion::GetMotor_PresentAngle(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_PresentAngle();
}

const float MotorUnion::GetMotor_PresentAngleSingleTurn(const unsigned char &idx) const
{
	float current_angle = Motor_Union.at(idx)->GetMotor_PresentAngle();
	float angle = fmod(current_angle, 360.0);

	if(angle < 0)
	{
		return 360.0 + angle;
	}
	else
	{
		return angle;
	}
}

const float &MotorUnion::GetMotor_PresentVelocity(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_PresentVelocity();
}

const float MotorUnion::GetMotor_PresentVelocityRPM(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_PresentVelocity() * Motor_Union.at(idx)->GetMotor_Scale2RPM();
}

const float &MotorUnion::GetMotor_PresentTorque(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_PresentTorque();
}

const bool &MotorUnion::GetMotor_PresentTorqueEnable(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_PresentTorqueEnable();
}

const bool &MotorUnion::GetMotor_MovingState(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Motor_MovingState();
}

const bool &MotorUnion::GetMotor_Arrival(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Arrival();
}
//------------------------------------------------------------------------------//
/*
	Set Motor Data
*/
void MotorUnion::SetMotor_Operating_Mode(const unsigned char &idx, char mode) const	//can't set mode online
{	
	// Motor_Union.at(idx)->SetMotor_TorqueEnable(false);
	// this_thread::sleep_for(chrono::milliseconds(50));
	Motor_Union.at(idx)->SetMotor_Operating_Mode(mode);
	// Motor_Union.at(idx)->WriteMode(mode);
	// this_thread::sleep_for(chrono::milliseconds(50));
	// Motor_Union.at(idx)->SetMotor_TorqueEnable(true);
}

void MotorUnion::SetMotor_CenterScale(const unsigned char &idx, const short &motor_center_scale) const
{
	Motor_Union.at(idx)->SetMotor_CenterScale(motor_center_scale);
}

void MotorUnion::SetMotor_Angle(const unsigned char &idx, const float &angle) const
{
	Motor_Union.at(idx)->SetMotor_Angle(angle);
	// if(idx + 1 != 7)
	// 	Motor_Union.at(idx+1)->SetMotor_Angle(angle);
}

void MotorUnion::SetMotor_Velocity(const unsigned char &idx, const int &velocity) const
{
	Motor_Union.at(idx)->SetMotor_Velocity(velocity);
}

void MotorUnion::SetMotor_Profile_Velocity(const unsigned char &idx, const int &velocity) const
{
	Motor_Union.at(idx)->SetMotor_Profile_Velocity(velocity);
}

void MotorUnion::SetMotor_Accel(const unsigned char &idx, const int &accel) const
{
	Motor_Union.at(idx)->SetMotor_Accel(accel);
}

void MotorUnion::SetMotor_AccelRPMM(const unsigned char &idx, const float &accel) const
{
	Motor_Union.at(idx)->SetMotor_Accel(int(accel/Motor_Union.at(idx)->GetMotor_Scale2RPMM()));
}

void MotorUnion::SetMotor_TorqueEnable(const unsigned char &idx, const bool &enable) const
{
	Motor_Union.at(idx)->SetMotor_TorqueEnable(enable);
}

void MotorUnion::SetMotor_Current(const unsigned char &idx, const int &current) const
{
	Motor_Union.at(idx)->SetMotor_Current(current);
}
////////////////////////////////////////////////////////////////////////////////
///   Background   /////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void MotorUnion::BGON()
{
	_is_deleted_thread_BG = false;
	thread_BG = new thread(&MotorUnion::BGReadWrite, this);
}

void MotorUnion::BGReadWrite()
{
	while (!_is_deleted_thread_BG)
	{
		WriteData();
		ReadData();
		this_thread::sleep_for(chrono::milliseconds(1));
	}
}

void MotorUnion::WriteData() const
{
	// Add parameters
	bool is_Write = false;
	for (int i = 0; i < Motor_Union.size(); i++)
		is_Write |= Motor_Union.at(i)->WriteData();

	// Write to motor
	if (is_Write)
	{
		for (int i = 0; i < groupBulkWrite.size(); i++)
		{
			int dxl_comm_result = groupBulkWrite.at(i)->txPacket();
			if (dxl_comm_result != COMM_SUCCESS)
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			groupBulkWrite.at(i)->clearParam();
		}
	}
}

void MotorUnion::ReadData() const
{
	// Add parameters
	for (int i = 0; i < Motor_Union.size(); i++)
		Motor_Union.at(i)->AddParam();

	// Read Data
	for (int i = 0; i < groupBulkRead.size(); i++)
	{
		int dxl_comm_result = groupBulkRead.at(i)->txRxPacket();
		if (dxl_comm_result != COMM_SUCCESS)
			printf("Motor %d, %s\n", Motor_Union.at(i) -> GetMotorID(), packetHandler->getTxRxResult(dxl_comm_result));
	}

	// Record to Motor
	for (int i = 0; i < Motor_Union.size(); i++)
		Motor_Union.at(i)->ReadData();

	// clear parameters
	for (int i = 0; i < groupBulkRead.size(); i++)
		groupBulkRead.at(i)->clearParam();
}