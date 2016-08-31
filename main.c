#define READ_PREAMBLE 0
#define READ_RESERVED 1
#define READ_LENGTH   2
#define READ_MESSAGE  3
#define READ_CHECKSUM 4

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

int checksumCounter;
int byteCounter;
int len;
int len1;
int raw_counter;
char raw_msg[1024];

int set_interface_attribs(int fd, int speed)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

void set_mincount(int fd, int mcount)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error tcgetattr: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5;        /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0)
		printf("Error tcsetattr: %s\n", strerror(errno));
}


int main()
{
	char *portname = "/dev/ttyUSB0";
	int fd;
	int wlen;

	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		return -1;
	}
	/*baudrate 115200, 8 bits, no parity, 1 stop bit */
	set_interface_attribs(fd, B9600);
	//set_mincount(fd, 0);
	/* set to pure timed read */

	/* simple output
    wlen = write(fd, "Hello!\n", 7);
    if (wlen != 7) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(fd); delay for output */


	/* simple noncanonical input */
	int status = 0;
	do {
		unsigned char buf[3];
		int rdlen;

		rdlen = read(fd, buf, sizeof(buf) - 1);
		if (rdlen > 0) {
			switch (status){
			case READ_PREAMBLE:
				if (((int) buf[0]) == 211){
					status = READ_RESERVED;
					raw_counter = 0;
					raw_msg[raw_counter] = buf[0];
					raw_counter++;
					//printf("PREAMBLE read\n");
					checksumCounter = 0;
					byteCounter = 0;
				}
				else{
					//printf("0x%x - %i\n", buf[0], (int) buf[0]);
				}
				break;
			case READ_RESERVED:
				raw_msg[raw_counter] = buf[0];
				raw_counter++;
				len1 = ((int) buf[0])  & 0x00000011;
				status = READ_LENGTH;
				//printf("RESERVED read\n");
				break;
			case READ_LENGTH:
				raw_msg[raw_counter] = buf[0];
				raw_counter++;
				len = (len1 << 8) + ((int) buf[0]) ;
				if(len==19) printf("\n");
				char message[100];
				char *msgptr = message;
				status = READ_MESSAGE;
				//printf("Length read: %i\n", len);
				break;
			case READ_MESSAGE:
				raw_msg[raw_counter] = buf[0];
				raw_counter++;
				// read
				message[byteCounter] = buf[0];
				if (byteCounter == (len - 1))
				{
					status = READ_CHECKSUM;

				}
				byteCounter++;
				break;
			case READ_CHECKSUM:
				raw_msg[raw_counter] = buf[0];
				raw_counter++;
				checksumCounter++;
				if(checksumCounter == 3)
				{
					// Check the message
					status = READ_PREAMBLE;
					// We've checked the message and it checks out
					// Check what message type it is
					unsigned char *b = message;
					int msg1 = *b;
					b++;
					int msg2 = *b;
					int RTK_msgtype= (msg1 << 4) + (msg2 >> 4);
					printf ("RTKmsgType: %i\n", RTK_msgtype);
					unsigned char   *p;
					for (p = message; byteCounter-- > 0; p++)
					{
						printf(" 0x%x", *p);
					}
					printf("\nRAW:\n");
					unsigned char* raw_ptr = raw_msg;
					for(int i = 0; i < raw_counter; i++)
					{
						printf(" 0x%x", *raw_ptr);
						raw_ptr++;
					}
					printf("\n");
				}
				break;
			}
/*
			//#ifdef DISPLAY_STRING
			buf[rdlen] = 0;
			printf("Read %d: \"%s\"\n", rdlen, buf);
			//#else  display hex
			unsigned char   *p;
			printf("Read %d:", rdlen);
			for (p = buf; rdlen-- > 0; p++)
				printf(" 0x%x", *p);
			printf("\n");
			//#endif
			 */
		} else if (rdlen < 0) {
			printf("Error from read: %d: %s\n", rdlen, strerror(errno));
		}
		/* repeat read to get full message */
	} while (1);
}

//int readBits(*char, start, len){
//}