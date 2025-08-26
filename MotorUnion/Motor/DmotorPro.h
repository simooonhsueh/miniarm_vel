#pragma once
#include "Dmotor.h"

class DmotorPro : public Dmotor
{
public:
	DmotorPro();
	DmotorPro(const unsigned char &MotorID, const string &MotorModel);
	~DmotorPro(){};
};