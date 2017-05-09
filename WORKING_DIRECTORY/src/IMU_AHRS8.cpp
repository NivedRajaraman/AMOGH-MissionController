/******
System libaries
******/

#include<iostream>


/*******
local Dependant libraries
*******/
#include "IMU_AHRS8.h"

AHRS8::AHRS8()
{
	float yaw, pitch, roll;
	float magX, magY, magZ;
	float aclX, aclY, aclZ;
	
	//Attitude variables intialisation
	this->yaw   = 0;
	this->pitch = 0;
	this->roll  = 0;
	
	this->mag[0]  = 0;
	this->mag[1]  = 0;
	this->mag[2]  = 0;
	
	this->acl[0]  = 0;
	this->acl[1]  = 0;
	this->acl[2]  = 0;
	
	this->gyro[0] = 0;
	this->gyro[1] = 0;
	this->gyro[2] = 0;
	
}

AHRS8::~AHRS8()
{
	
}

bool AHRS8::serialInitialize(std::string serial_port)
{

	return this->serial.open(serial_port,115200,8,boost::asio::serial_port::parity::none, boost::asio::serial_port_base::stop_bits::one,boost::asio::serial_port::flow_control::none);

}



void AHRS8::loadScripts()
{
	//Script to begin continous fetching of data variables
	std::string IMU_function="forget quatCheck\r : quatCheck begin ?key 0= while magp di. accelp di. gyrop di. cr yaw di. pitch di. roll di. cr 100 delay repeat ;\r ";

	this->serial.IMU_send_command(IMU_function);
	
	//Command to start begin script
	char send_text[]="quatCheck\r";
	this->serial.send(send_text,sizeof(send_text)-1);

}

float AHRS8::getAttitude(std::string attitude)
{
	char *token = std::strtok((char*)attitude.c_str(), " ");
	int i=0;
	    while (token != NULL) 
	    {
		    if(i==2)
		    {
		    	//std::cout<<"Attitude--" << atof(token) << '\n';
		    	return atof(token);
		    }
			
			token = std::strtok(NULL, " ");
			i++;
	    }
}

float AHRS8::getVariants(std::string variant)
{
	char* string_charArray=(char*)variant.c_str();
	char *token = std::strtok((char*)variant.c_str(), " ");
	int i=0, value_position=0;
	
	if(string_charArray[0]!='0')
	{
		value_position=3;
	}
	else
	{
		value_position=1;
	}
	
	while (token != NULL)
	    {
		if(i==value_position)
		{
			//std::cout<<"Variant-- " << atof(token) << '\n';
			return atof(token);
		}
		token = std::strtok(NULL, " ");
		i++;
	    }
   	
   	
   	
}

void AHRS8::parseDataPacket(char* dataPacket)
{
	std::string dataInstr[20];
	int k =0;
	for(int i =0; i< 12; i++){
		while(dataPacket[k] != char(10)){
			dataInstr[i].push_back(dataPacket[k]);
			k++;
		}
		while(dataPacket[k] == char(10) || dataPacket[k] == char(13) || dataPacket[k] == char(0)){
			k++;
		}
	}
	   
	std::string temp;
	//printf("str.......\n");
	
	for(int i=0; i<12; i++)
	{
		char* attribute = (char*)dataInstr[i].c_str();
		//std::cout<<"attribute[0] i--- "<<attribute[0]<<std::endl;
		switch(attribute[0])
		{
			case 'y':
				this->yaw=this->getAttitude(dataInstr[i]);
				break;
			
			case 'p':
				this->pitch=this->getAttitude(dataInstr[i]);
				break;
			
			case 'r':
				this->roll=this->getAttitude(dataInstr[i]);
				break;
			
			case 'm':
				for(int j=0;j<3;j++)
				{
					this->mag[j]=this->getVariants(dataInstr[i]);
					i++;
				}
				i--;
				break;
			
			case 'a':
				for(int j=0;j<3;j++)
				{
					this->acl[j]=this->getVariants(dataInstr[i]);
					i++;
				}
				i--;
				break;
			case 'g':
				for(int j=0;j<3;j++)
				{
					this->gyro[j]=(GYRO_CONSTANT)*(this->getVariants(dataInstr[i]));
					i++;
				}
				i--;
				break;
			
			default:
				break;
			
		}
		

	}
	
}

void AHRS8::fetchSerialData(STATE* shm)
{
	char* dataPacket;
	std::string serialResponse;
	
	this->loadScripts();
	
	while(1)
	{
		std::string received_text=this->serial.IMU_receive();

		dataPacket=(char*)received_text.c_str();

		this->parseDataPacket(dataPacket);

		shm->_roll = this->yaw;
		shm->_pitch = this->pitch;
		shm->_yaw = this->yaw;
		shm->_gyroX = this->gyro[0];
		shm->_gyroY = this->gyro[1];
		shm->_gyroZ = this->gyro[2];
	}
	
	
}

float AHRS8::getHeading()
{
	return (this->yaw);
}

float AHRS8::getPitch()
{
	return (this->pitch); 
}

float AHRS8::getRoll()
{
	return (this->roll);
}

