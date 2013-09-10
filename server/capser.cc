/* capser.c
	Helper functions for "ser"
*/
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

int writeport(int fd, char *chars) {
	int len = strlen(chars);
	//printf("writeport: %s\n",chars);
	if (strchr(chars,'\r') - chars != len-1)
		printf("writeport: did not find \\r as last character of output");
	int n = write(fd, chars, len);
	if (n != len) {
		fputs("write failed!\n", stderr);
		return 0;
	}
	return 1;
}

int readport(int fd, char *result) {
	int iIn = read(fd, result, 254);
	if (iIn < 0) {
		if (errno == EAGAIN) {
			printf("SERIAL EAGAIN ERROR\n");
			result[0] = '\0';
			return -2;
		} else {
			printf("SERIAL read error %d %s\n", errno, strerror(errno));
			return 0;
		}
	}
	result[iIn] = 0x00;
	//printf("readport: %s\nend readport\n",result);
	return 1;
}

int readport_blocking(int fd, char *result, char *end) {
	int end_len = strlen(end);
	int read_count = 0;
	int iIn;
	while (true) {
		//printf("ready to read\n");
		iIn  = read(fd, result+read_count, 1);
		//printf("read %d chars: %d\n",iIn,result[read_count]);
		if (iIn != 1) {
			if (errno == EAGAIN) {
				printf("SERIAL EAGAIN ERROR\n");
				result[0] = '\0';
				return -2;
			} else {
				printf("SERIAL read error %d %s\n", errno, strerror(errno));
				return 0;
			}
		}
		read_count++;
		if (read_count > end_len && strncmp(result+(read_count-end_len),end,end_len)==0) {
			break;
		}
	}
	result[read_count] = 0x00;
	//printf("readport: %s\nend readport\n",result);
	return 1;
}

int getbaud(int fd) {
	struct termios termAttr;
	int inputSpeed = -1;
	speed_t baudRate;
	tcgetattr(fd, &termAttr);
	/* Get the input speed.                              */
	baudRate = cfgetispeed(&termAttr);
	switch (baudRate) {
		case B0:      inputSpeed = 0; break;
		case B50:     inputSpeed = 50; break;
		case B110:    inputSpeed = 110; break;
		case B134:    inputSpeed = 134; break;
		case B150:    inputSpeed = 150; break;
		case B200:    inputSpeed = 200; break;
		case B300:    inputSpeed = 300; break;
		case B600:    inputSpeed = 600; break;
		case B1200:   inputSpeed = 1200; break;
		case B1800:   inputSpeed = 1800; break;
		case B2400:   inputSpeed = 2400; break;
		case B4800:   inputSpeed = 4800; break;
		case B9600:   inputSpeed = 9600; break;
		case B19200:  inputSpeed = 19200; break;
		case B38400:  inputSpeed = 38400; break;
	}
	return inputSpeed;
}
