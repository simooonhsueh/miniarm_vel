#pragma once
#include "Dmotor.h"

class DmotorXm : public Dmotor
{
public:
	DmotorXm();
	DmotorXm(const unsigned char &MotorID, const string &MotorModel);
	~DmotorXm(){};
};