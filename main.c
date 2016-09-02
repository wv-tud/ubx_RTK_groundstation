/*
[Termios reference:                http://man7.org/linux/man-pages/man3/termios.3.html]
[File handling reference:          http://www.gdsw.at/languages/c/programming-bbrown/c_075.htm]
[RTCM reference:                   http://www.rtcm.org/overview.php#Standards]
[RTCM 10403.2 reference:           https://www.u-blox.com/sites/default/files/NEO-M8P_DataSheet_(UBX-15016656).pdf]
[RTCM Structure reference:         https://github.com/wv-tud/ubx_RTK_groundstation/blob/master/087_rubinov_collier_fuller_seager.pdf]
[RTCM message content reference:   https://github.com/tomojitakasu/RTKLIB/blob/master/src/rtcm3.c]
[Ivybus reference:                 https://github.com/paparazzi/paparazzi/blob/a99dfbd5e8ab94f6e227b0e8daca1cf7677b088e/sw/ground_segment/misc/sbp2ivy.c]
[1005 reference:                   https://github.com/jcmb/RTCM3/blob/master/DEFS/1005.RTCM3]
*/

#ifndef GPS_PORT
#define GPS_PORT "/dev/ttyUSB0"
#endif

#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include "rtcm3.h"

struct EcefCoor_f {
  float x; ///< in meters
  float y; ///< in meters
  float z; ///< in meters
};

struct LlaCoor_f {
  float lat; ///< in radians
  float lon; ///< in radians
  float alt; ///< in meters (normally above WGS84 reference ellipsoid)
};

struct RTCM3_1005 {
	int StaId;
	int ItRef;
	int indGPS;
	int indGlonass;
	int indGalileo;
	int indRefS;
	struct EcefCoor_f posEcef;
	struct LlaCoor_f posLla;
};

struct RTCM3_msg {
	int length;
	int type;
	unsigned char raw_msg[1024];
	struct RTCM3_1005 *msg1005;
};

struct LlaCoor_f lla_of_ecef_f(struct EcefCoor_f);
int              set_interface_attribs(int, int);
void             set_mincount(int, int);
unsigned int     getbitu(unsigned char *, int , int );
int              getbits(unsigned char *, int , int );
static double    getbits_38(unsigned char *, int );
void             readMessage(int *, struct RTCM3_msg *);

// Required global variables
int fd;

#define MSG_DEST	"ground"
#define MSG_NAME    "FLIGHT_PARAM"
#define MSG_ID		"GCS"


static uint32_t sbp_read(uint8_t *buff, uint32_t n, void *context __attribute__((unused)))
{
  int ret = read(fd, buff, n);
  if(ret > 0)
    return ret;
  else
    return 0;
}


int main()
{
	// Initialize variables
	// Set GPS port
	char *portname = GPS_PORT;
	//char* ivy_bus;
	//ivy_bus = "127.255.255.255";
	// Open serial port
	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		return -1;
	}
	/*baudrate 9600, 8 bits, no parity, 1 stop bit */
	set_interface_attribs(fd, B9600);
	//set_mincount(fd, 0);
	/* set to pure timed read */

	/* simple output */
	/*
    int wlen = write(fd, "Hello!\n", 7);
    if (wlen != 7) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(fd); delay for output
	 */
	//IvyInit ("GPSd2Ivy", "GPSd2Ivy READY", NULL, NULL, NULL, NULL);
	//IvyStart(ivy_bus);

	/* simple noncanonical input */
	sbp_state_t state;
	int readstatus;
	while ( 1 == 1 )
	{
		readstatus = sbp_process(&state, &sbp_read);
		if (readstatus ==  SBP_OK_CALLBACK_EXECUTED){

		printf("Read message %i of length %i\n", state.msg_type, state.msg_len);
		if (state.msg_type == 1005){
			struct RTCM3_1005 msg1005;

			msg1005.StaId 		= getbitu(state.msg_buff, 24 + 12, 12);
			msg1005.ItRef 		= getbitu(state.msg_buff, 24 + 24, 6);
			msg1005.indGPS 		= getbitu(state.msg_buff, 24 + 30, 1);
			msg1005.indGlonass 	= getbitu(state.msg_buff, 24 + 31, 1);
			msg1005.indGalileo 	= getbitu(state.msg_buff, 24 + 32, 1);
			msg1005.indRefS 	= getbitu(state.msg_buff, 24 + 33, 1);
			msg1005.posEcef.x   = getbits_38(state.msg_buff, 24 + 34) * 0.0001;
			msg1005.posEcef.y   = getbits_38(state.msg_buff, 24 + 74) * 0.0001;
			msg1005.posEcef.z   = getbits_38(state.msg_buff, 24 + 114) * 0.0001;
			msg1005.posLla      = lla_of_ecef_f(msg1005.posEcef);

			printf("To send: (Lat: %f) \t (Lon: %f) \t (Alt: %f)\n", msg1005.posLla.lat / (2*3.1415) * 360, msg1005.posLla.lon / (2*3.1415) * 360, msg1005.posLla.alt);
			printf("StaId: %i \n", msg1005.StaId);
			printf("ItRef: %i \n", msg1005.ItRef);
			printf("indGPS: %i \n", msg1005.indGPS);
			printf("indGlonass: %i \n", msg1005.indGlonass);
			printf("indGallileo: %i \n", msg1005.indGalileo);
			printf("indRefS: %i \n", msg1005.indRefS);

		}
			/*
		IvySendMsg("%s %s %s %f %f %f %f %f %f %f %f %f %f %f %d %f",
				MSG_DEST,
				MSG_NAME,
				MSG_ID, // ac_id
				0.0, // roll,
				0.0, // pitch,
				0.0, // heading
				llh[0],
				llh[1],
				0,
				0, // course
				llh[2],
				0,
				0.0, // agl
				0,
				0, // itow
				0.0); // airspeed
			 */
		}
		else if (readstatus==SBP_CRC_ERROR){
			printf("Error: Resetting status to Preamble \n");
			state.state = READ_PREAMBLE;
		}
	}
	return 0;
}

//void  readMessage(int *fd, struct RTCM3_msg *message){
//	int len = 0;
//	int len1 = 0;
//	do {
//		unsigned char buf[3];
//		int rdlen;
//		rdlen = read(*fd, buf, sizeof(buf) - 1);
//		if (rdlen > 0) {
//			switch (status){
//			case READ_PREAMBLE:
//				if (((int) buf[0]) == 211){
//					status = READ_RESERVED;
//					rawIndex = 0;
//					message->raw_msg[rawIndex] = buf[0];
//					checksumCounter = 0;
//					byteIndex = 0;
//				}
//				break;
//			case READ_RESERVED:
//				message->raw_msg[rawIndex] = buf[0];
//				len1 = ((int) buf[0])  & 0b00000011;
//				status = READ_LENGTH;
//				break;
//			case READ_LENGTH:
//				message->raw_msg[rawIndex] = buf[0];
//				len = (len1 << 8) + ((int) buf[0]) ;
//				status = READ_MESSAGE;
//				break;
//			case READ_MESSAGE:
//				message->raw_msg[rawIndex] = buf[0];
//				if (byteIndex == (len - 1)) status = READ_CHECKSUM;
//				byteIndex++;
//				break;
//			case READ_CHECKSUM:
//				message->raw_msg[rawIndex] = buf[0];
//				checksumCounter++;
//				if(checksumCounter == 3)
//				{
//					// Check the message
//					status = READ_PREAMBLE;
//					// We've checked the message and it checks out
//					// Check what message type it is
//					message->type   = getbitu(message->raw_msg, 24 + 0, 12);
//					message->length = rawIndex+1;
//					// Printing useful messages from RTK1005 msg.
//					return;
//				}
//				break;
//			}
//			rawIndex++;
//		} else if (rdlen < 0) {
//			printf("Error from read: %d: %s\n", rdlen, strerror(errno));
//		}
//		/* repeat read to get full message */
//	} while (1);
//}

struct LlaCoor_f lla_of_ecef_f(struct EcefCoor_f posecefpos)
{
	struct LlaCoor_f posllapos;
	// FIXME : make an ellipsoid struct
	static const float a = 6378137.0;           /* earth semimajor axis in meters */
	static const float f = 1. / 298.257223563;  /* reciprocal flattening          */
	const float b = a * (1. - f);               /* semi-minor axis                */
	const float b2 = b * b;

	const float e2 = 2.*f - (f * f);            /* first eccentricity squared     */
	const float ep2 = f * (2. - f) / ((1. - f) * (1. - f)); /* second eccentricity squared    */
	const float E2 = a * a - b2;


	const float z2 = posecefpos.z * posecefpos.z;
	const float r2 = posecefpos.x * posecefpos.x + posecefpos.y * posecefpos.y;
	const float r = sqrtf(r2);
	const float F = 54.*b2 * z2;
	const float G = r2 + (1 - e2) * z2 - e2 * E2;
	const float c = (e2 * e2 * F * r2) / (G * G * G);
	const float s = powf((1 + c + sqrtf(c * c + 2 * c)), 1. / 3.);
	const float s1 = 1 + s + 1 / s;
	const float P = F / (3 * s1 * s1 * G * G);
	const float Q = sqrtf(1 + 2 * e2 * e2 * P);
	const float ro = -(e2 * P * r) / (1 + Q) + sqrtf((a * a / 2) * (1 + 1 / Q) - ((1 - e2) * P * z2) / (Q *
			(1 + Q)) - P * r2 / 2);
	const float tmp = (r - e2 * ro) * (r - e2 * ro);
	const float U = sqrtf(tmp + z2);
	const float V = sqrtf(tmp + (1 - e2) * z2);
	const float zo = (b2 * posecefpos.z) / (a * V);

	posllapos.alt = U * (1 - b2 / (a * V));
	posllapos.lat = atanf((posecefpos.z + ep2 * zo) / r);
	posllapos.lon = atan2f(posecefpos.y, posecefpos.x);

	return posllapos;
}

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

unsigned int getbitu(unsigned char *buff, int pos, int lenb)
{
	unsigned int bits=0;
	int i;
	for (i=pos;i<pos+lenb;i++) bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);
	return bits;
}

int getbits(unsigned char *buff, int pos, int lenb)
{
	unsigned int bits=getbitu(buff,pos,lenb);
	if (lenb<=0||32<=lenb||!(bits&(1u<<(lenb-1)))) return (int)bits;
	return (int)(bits|(~0u<<lenb)); /* extend sign */
}

static double getbits_38(unsigned char *buff, int pos)
{
	return (double)getbits(buff,pos,32)*64.0+getbitu(buff,pos+32,6);
}
