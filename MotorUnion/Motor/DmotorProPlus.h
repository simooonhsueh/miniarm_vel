#pragma once
#include "Dmotor.h"

class DmotorProPlus : public Dmotor
{
public:
	DmotorProPlus();
	DmotorProPlus(const unsigned char &MotorID, const string &MotorModel);
	~DmotorProPlus(){};
};