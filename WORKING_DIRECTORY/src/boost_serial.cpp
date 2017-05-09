#include "boost_serial.h"


Serial::Serial() : m_serialPort(m_io) {
 m_io.run();
}
 
Serial::~Serial() {
  this->m_serialPort.close();

}

bool Serial::open(const std::string& portname, 
                  int baudrate, 
                  int charactersize, 
                  boost::asio::serial_port_base::parity::type parity, 
                  boost::asio::serial_port_base::stop_bits::type stopbits,
                  boost::asio::serial_port_base::flow_control::type flowcontrol)
{
 this->m_serialPort.open(portname.c_str());
  if(!this->m_serialPort.is_open())
 {
 	return false;
 }
 this->m_serialPort.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
 this->m_serialPort.set_option(boost::asio::serial_port_base::character_size(charactersize));
 this->m_serialPort.set_option(boost::asio::serial_port_base::parity(parity));
 this->m_serialPort.set_option(boost::asio::serial_port_base::stop_bits(stopbits));
 this->m_serialPort.set_option(boost::asio::serial_port_base::flow_control(flowcontrol));
 
 return true;
}


bool Serial::isOpen()
{
	return this->m_serialPort.is_open();
}


void Serial::close()
{
  this->m_serialPort.close();
}

std::string Serial::receive(int receive_length)
{
 using namespace boost;
        char c;
        std::string result;
        bool print = false;
        int char_count=0;
	for(int i=0;i<receive_length;i++)
	{
		asio::read( this->m_serialPort,asio::buffer(&c,1));
	    
                    result+=c;
                   
         }

	return result;

}


std::string Serial::IMU_receive()
{
	
 using namespace boost;
        char c;
        std::string result;
        bool print = false;
        int char_count=0;
        
   while(1)
   {	c='\0';
            asio::read( this->m_serialPort,asio::buffer(&c,1));
            
            if(c=='m')
            {
            	if(print)
            	{
            		result+=c;
            		break;
            	}
            	else
            	{
            		print=true;
            	}
            }
		
		if(print)
		{	
			result+=c;
			char_count++;

			//printf("%c",c);
			
//			if(c=='\n')
//			{
//				printf("the Count=%d--",char_count);
//			}
			
		}

		
	}

	return result;

}
size_t Serial::send(void* data, size_t length)
{
 return boost::asio::write( this->m_serialPort,boost::asio::buffer(data, length));
}


void Serial::IMU_send_command(std::string command)
{	
	char* dummy_command = (char*)command.c_str();
	this->send(dummy_command,strlen(dummy_command)-1);
	this->receive(strlen(dummy_command));
	
	
}

