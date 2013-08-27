/* ser.c
	(C) 2004-5 Captain http://www.captain.at
	
	Sends 3 characters (ABC) via the serial port (/dev/ttyS0) and reads
	them back if they are returned from the PIC.
	
	Used for testing the PIC-MMC test-board
	http://www.captain.at/electronic-index.php

*/

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include "capser.h"

int fd;

int initport(int fd) {
	struct termios options;
	// Get the current options for the port...
	tcgetattr(fd, &options);
	// Set the baud rates...
	int reply = cfsetispeed(&options, B1200);
	
	//int reply = cfsetispeed(&options, B57600);
	printf("error = %d\n", reply);
	reply = cfsetospeed(&options, B1200);
	
	//reply = cfsetospeed(&options, B57600);
	printf("error = %d\n", reply);
	// Enable the receiver and set local mode...
	options.c_cflag |= (CLOCAL | CREAD);

	//options.c_lflag |= ~(ICANON | ECHO | ECHOE);
	
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~-CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	// Set the new options for the port...
	tcsetattr(fd, TCSANOW, &options);
	return 1;
}

int main(int argc, char **argv) {

	printf("OPENING USB PORT\n");
	//fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		perror("open_port: Unable to open port - ");
		return 1;
	} else {
		fcntl(fd, F_SETFL, 0);
	}
	
	printf("baud=%d\n", getbaud(fd));
	initport(fd);
	printf("baud=%d\n", getbaud(fd));

	char sCmd[50];//={"remote"};
	sCmd[0] = 114;
	sCmd[1] = 101;	
	sCmd[2] = 109;
	sCmd[3] = 111;
	sCmd[4] = 116;
	sCmd[5] = 101;
	sCmd[6] = 13;
	sCmd[7] = 10;
	sCmd[8] = 0;	

	if (!writeport(fd, sCmd)) {
		printf("write failed\n");
		close(fd);
		return 1;
	}

	printf("written:%s\n", sCmd);
	
	usleep(500000);
	char sResult[254];
	fcntl(fd, F_SETFL, FNDELAY); // don't block serial read

	if (!readport(fd,sResult)) {
		printf("read failed\n");
		close(fd);
		return 1;
	}
	printf("readport=%s\n", sResult);
	close(fd);
	return 0;
}
