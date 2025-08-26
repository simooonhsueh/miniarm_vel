#pragma once
#include "Dmotor.h"

class DmotorMx : public Dmotor
{
public:
	DmotorMx();
	DmotorMx(const unsigned char &MotorID, const string &MotorModel);
	~DmotorMx(){};
};