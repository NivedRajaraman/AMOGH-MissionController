/******
System libaries
******/

#include<iostream>


/*******
local Dependant libraries
*******/

using namespace std;
#include "IMU_AHRS8.h"


int main(int argc, char** argv)
{
	AHRS8 IMU;
	if(IMU.serialInitialize("/dev/ttyUSB0"))
	{
		cout<<"Port Open Successful"<<endl;
	}
	else
	{
		cout<<"Please check the connection"<<endl;
	}
	
	IMU.fetchSerialData();

	
	return 0;
}
