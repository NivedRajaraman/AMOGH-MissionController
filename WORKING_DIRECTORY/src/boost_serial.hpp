#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>
#include <boost/asio.hpp>

/**
 * @class Serial
 * @brief Supports serial port communication.
*/
class Serial
{
public:
 Serial();
 ~Serial();
 
 /// Opens a connection to the serial port.
 /**
  * The parity, stop bits and flowcontrol are types of the boost::asio::serial_port_base.
  * @param portname The device name.
  * @param baudrate The Baudrate.
  * @param charactersize The Character size in bits.
  * @param parity The parity.
  * @param stopbits The stop bits.
  * @param flowcontrol The flowcontrol
 */
 bool open(const std::string& portname, 
           int baudrate, 
           int charactersize, 
           boost::asio::serial_port_base::parity::type parity, 
           boost::asio::serial_port_base::stop_bits::type stopbits,
           boost::asio::serial_port_base::flow_control::type flowcontrol);
           
           
           
 bool isOpen();
  
 /// Close the serial port.
 void close();
  
 /// Receive data from the serial port.
 /**
  * @param data The pointer to the data which will be filled.
  * @param length The number of data to read. (In Bytes)
  * @return The number of received data.
  */
std::string receive(int receive_command_length);
std::string IMU_receive();
 
 /// Send data to the serial port.
 /**
  * @param data The pointer to the data which will be send.
  * @param length The number of data to send. (In Bytes)
  * @return The number of transfered data.
  */
 size_t send(void* data, size_t length);
void IMU_send_command(std::string command);
private:
 boost::asio::io_service m_io;
 boost::asio::serial_port m_serialPort;
};
#endif // SERIALPORT_H
