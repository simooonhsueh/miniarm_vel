#pragma once
#include "./Motor/DmotorMx.h"
#include "./Motor/DmotorPro.h"
#include "./Motor/DmotorProPlus.h"
#include "./Motor/DmotorXm.h"
#include <vector>
#include <thread>
#include <chrono>

class MotorUnion
{
public:
	/* 
	@ ID, 
	@ MotorModel, 
	@ Port
	*/
	MotorUnion(const vector<unsigned char> &IDArray,
			   const vector<string> &MotorModelArray);
	virtual ~MotorUnion();
	void WaitMotorArrival(int i) const;
private:
	template <class T>
	void deleteInVector(vector<T *>);

	////////////////////////////////////////////////////////////////////////////////
	///  All Motors   //////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////
private:
	/* Control All Motors */
	const bool ConnectAllMotors(vector<unsigned char> &AllPortNumber);
	const bool CheckAllMotorsConnected() const;

	/* Get All Motors Data */
	const bool GetAllMotorsTorqueEnable() const;

protected:
	/* Wait */
	void WaitAllMotorsArrival() const;
	const bool CheckAllMotorsArrival() const;
	void WaitAllMotorsArrival(const int &total_waiting_time_ms) const;
	void StopMotor(int idx) const;
	void StopAllMotors() const;

	/* Set All Motors Data */
	void SetAllMotorsOperatingMode(const unsigned char &mode) const;
	void SetAllMotorsAngle(const float &angle) const;
	void SetAllMotorsVelocity(const int &velocity) const;
	void SetAllMotorsProfileVelocity(const int &velocity) const;
	void SetAllMotorsAccel(const int &accel) const;
	void SetAllMotorsTorqueEnable(const bool &enable) const;

	////////////////////////////////////////////////////////////////////////////////
	///   Motor   //////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////
	/* Get Motor Data */
public:
	const size_t GetMotor_Number() const;
	const unsigned char &GetMotor_ID(const unsigned char &idx) const;
	const bool &GetMotor_Connected(const unsigned char &idx) const;
	const bool &GetMotor_TorqueEnable(const unsigned char &idx) const;
	const float &GetMotor_PresentAngle(const unsigned char &idx) const;
	const float GetMotor_PresentAngleSingleTurn(const unsigned char &idx) const;
	const float &GetMotor_PresentTorque(const unsigned char &idx) const;
	const float &GetMotor_PresentVelocity(const unsigned char &idx) const;
	const bool &GetMotor_PresentTorqueEnable(const unsigned char &idx) const;
	const float GetMotor_PresentVelocityRPM(const unsigned char &idx) const;
	const bool &GetMotor_MovingState(const unsigned char &idx) const;
	const bool CheckPort_Connected() const;
	/* Get Any Motors Data */
	const bool GetAnyMotorsTorqueEnable() const;
	void SetTerminated_flag(bool *terminated);
	bool GetMotor_Terminated(void) const;

	const float &GetMotor_Scale2RPM(const unsigned char &idx) const;
	const float &GetMotor_Scale2RPMM(const unsigned char &idx) const;
	const short &GetMotor_CenterScale(const unsigned char &idx) const;
	const float &GetMotor_Angle(const unsigned char &idx) const;
	const int &GetMotor_Velocity(const unsigned char &idx) const;
	const int &GetMotor_Accel(const unsigned char &idx) const;
	const bool &GetMotor_Arrival(const unsigned char &idx) const;
protected:
	/* Set Motor Data */
	void SetMotor_Operating_Mode(const unsigned char &idx, char mode) const;
	void SetMotor_CenterScale(const unsigned char &idx, const short &motor_center_scale) const;
	void SetMotor_Angle(const unsigned char &idx, const float &angle) const;
	void SetMotor_Velocity(const unsigned char &idx, const int &velocity) const;
	void SetMotor_Profile_Velocity(const unsigned char &idx, const int &velocity) const;
	void SetMotor_Accel(const unsigned char &idx, const int &accel) const;
	void SetMotor_AccelRPMM(const unsigned char &idx, const float &accel) const;
	void SetMotor_TorqueEnable(const unsigned char &idx, const bool &enable) const;
	void SetMotor_Current(const unsigned char &idx, const int &current) const;
	bool *terminated_;

	////////////////////////////////////////////////////////////////////////////////
	///   Background   /////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////
private:
	//Background is used for reading & writing data to motor
	thread *thread_BG;
	bool _is_deleted_thread_BG;

	vector<dynamixel::PortHandler *> portHandler;
	dynamixel::PacketHandler *packetHandler;
	vector<dynamixel::GroupBulkRead *> groupBulkRead;
	vector<dynamixel::GroupBulkWrite *> groupBulkWrite;

	void BGON();
	void BGReadWrite();
	void WriteData() const;
	void ReadData() const;

private:
	vector<Dmotor *> Motor_Union;
	const int waiting_frequency;

public:
	static vector<unsigned char> allport;
};
