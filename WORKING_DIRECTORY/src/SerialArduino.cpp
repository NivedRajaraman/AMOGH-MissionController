#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */
#include <iostream>
#include <stdlib.h>
#include <string.h>


int openPortToArduino()
{
    int fd,fd1;
    int wr = 0;
    int re = 0;
    int wr1 = 0;
    int re1 = 0;/*File Descriptor*/

		
	printf("\n +----------------------------------+");
	printf("\n |        Serial Port               |");
	printf("\n +----------------------------------+");
	printf("\n");

	/*------------------------------- Opening the Serial Port -------------------------------*/

	/* Change /dev/ttyUSB0 to the one corresponding to your system */

    fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NDELAY);
        		/* /dev/ttyUSB0 is the FT232 based USB2SERIAL Converter   */
			   					/* O_RDWR   - Read/Write access to serial port       */
								/* O_NOCTTY - No terminal will control the process   */
								/* Open in blocking mode,read will wait              */
			
							                                        
	if(fd == -1)						/* Error Checking */
        printf("\n  Error! in Opening ttyUSB0 \n");
	else
    	printf("\n  ttyUSB0 Opened Successfully \n");

	
		/*---------- Setting the Attributes of the serial port using termios structure --------- */
		
	struct termios SerialPortSettings;	/* Create the structure                          */

	tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

		/* Setting the Baud rate */
	cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 9600                       */
	cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 9600                       */

		/* 8N1 Mode */
	SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
		
	SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
		
		
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

	SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/
		
		/* Setting Time outs */
	SerialPortSettings.c_cc[VMIN] = 1; /* Read at least 1 characters */
	SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */


	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
		printf("\n  ERROR ! in Setting attributes \n");
	else
        printf("\n  BaudRate = 115200   StopBits = 1   Parity = none \n");

    printf("\n +----------------------------------+\n\n\n");
		
	/*------------------------------- Now can read data from serial port -----------------------------*/
	return fd;
}
        

void closePortToArduino(int fd)        
{
	close(fd)
}
