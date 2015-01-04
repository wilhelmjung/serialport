//
// Non-Canonical Input Processing,i.e, Raw Mode
// @file: uart.c
//
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <strings.h>
#include <string.h>

#include "uart.h"

//default config for serial port
static struct uart_config default_uart_config = {
	.baudrate	= DEFAULT_BAUDRATE, // 9600
	.data		= 8,	// 5,6,7,8,
	.parity		= 'e',	// 'n'one, 'e'ven(PARENB), 'o'dd(PARODD)
	.stop		= 1,	// stop bits: 1,2
	.rtscts		= 0	// output HW flow ctrl (0,1 / no,yes)
};

//forwad declaration of private functions
static void         set_parity  (struct termios *options, char parity);
static u_int inline int2baudrate(int rate);
static u_int inline int2databits(int bits);

////////////////////////////////////PUBLIC/////////////////////////////////////
int uart_open(const char *path, int *fdp, struct uart_config *uc)
{
	struct termios newtio;

	if (! uc)
		uc = &default_uart_config;

fprintf(stderr, "SerialPort: %s %i,%i-%c-%i\r\n",
	path, uc->baudrate, uc->data, uc->parity, uc->stop);

	*fdp = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);//|O_NDELAY|O_SYNC
	if (*fdp < 0) {
		perror(path);
		return -1;
	}

	// 0. Save old termios options
	//tcgetattr(fd, &oldtio); /* save current port settings */

	// 1. Get a new option
	memset(&newtio, 0, sizeof(struct termios));

	/**
	 * parity enable, enable the receiver,local mode
	 * BAUDRATE: 9600, CS8: 8E1(8 data bits,even parity,1 stop bits).
	 * No HW flow control
	 */
	// 2. Set raw mode(for input&output),must set local mode, enable recv
	newtio.c_cflag |= (CLOCAL | CREAD);
	//newtio.c_cflag &= CSIZE; // mask for CS8
#if 0
	newtio.c_cflag |= CS8; //8 data bits, default
#else
	newtio.c_cflag |= int2databits(uc->data); // u_char: 5,6.7,8
	if (uc->rtscts) {
		newtio.c_cflag |= CRTSCTS; // HW flow ctrl
	}
#endif

#if 0
	newtio.c_cflag |= int2baudrate(baud);// BXXX
#else
	cfsetspeed(&newtio, int2baudrate(uc->baudrate));
#endif

	//Set bauderate for both input and output
	//XXX line ctrl characters (to be deleted) 
	//newtio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN) ;
	//Local Options: Choosing Raw Input
	//newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	// XXX set input mode (to be deleted )
	// (non-canonical, no echo,...) raw input mode
	//newtio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP);
	//newtio.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON);
	//XXX (to be deleted) set output mode

#if 0
	newtio.c_oflag &= ~OPOST; //raw data output(no postprocess for output)
#else
	newtio.c_oflag = 0;	//No options for output processing
#endif
	// 3. Set parity
#if 0
//	newtio.c_cflag |= PARENB;	/* Enable parity */
//	newtio.c_cflag &= ~CSTOPB;	/* 1 stop bit */
//	newtio.c_cflag &= ~PARODD;	/* change to even parity */
//	newtio.c_iflag |= (INPCK|ISTRIP); //XXX  !! input parity check
#else	// 'n','o','e'; input parity bit check&strip
	set_parity(&newtio, uc->parity); // none,odd,even
#endif
	//don't wait for incoming chars
	newtio.c_cc[VTIME] = 0;   // inter-character timer not used
	newtio.c_cc[VMIN]  = 0;   //FIXME blocking read until n chars recved

	tcflush(*fdp, TCIFLUSH);  // discard input but not read
	//apply new termio option, imediately
	tcsetattr(*fdp, TCSANOW, &newtio);

	return 0;
}

//Just modify serial port settings
int
uart_set(const char *path, int *fdp, struct uart_config *uc)
{
	int ret = uart_open(path, fdp, uc);
	if (-1 == ret) {
		fprintf(stderr, "Config serial port failed1\n");
		return -1;
	}

	ret = uart_close(fdp);
	if (-1 == ret) {
		fprintf(stderr, "Config serial port failed2\n");
		return -1;
	}

	fprintf(stderr, "Config serial port OK!\n");

	return 0;
}

int
uart_close(int *fdp)
{
	int ret = close(*fdp);
	//tcsetattr(fd, TCSANOW, &oldtio);
	if (ret != 0) {
		perror("Uart_colse:");
		return -1;
	}

	return 0;
}

int
uart_write(int *fdp, char *buf, int len)
{
	int ret;
	ret = write(*fdp, buf, len);
	/* XXX sync write, waits untill all output written to
	 * the object referred to by fd has been transmitted.
	 * tcdrain() is a must have,othrewise data output from uart may lost!
	 */
	if (ret != len)
		perror("uart_write:");
	tcdrain(*fdp);

	return ret;
}

int
uart_read(int *fdp, char *buf, int len)
{
	int ret;
	ret = read(*fdp, buf, len);
	return ret;
}

////////////////////////////////// PRIVATE /////////////////////////////////////
#ifdef CONFIG_UART_RAW_MODE
static void
set_raw_mode(struct termios *options) //conflict with set_parity()
{
	options->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
				INLCR  | IGNCR  | ICRNL  | IXON);
	//XXX don't enable INPCK and ISTRIP,even even parity enabled.
	options->c_oflag &= ~OPOST;
	options->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options->c_cflag &= ~(CSIZE | PARENB);
	options->c_cflag |= CS8;
}
#endif

static void
set_parity(struct termios *options, char parity)
{
	switch (parity) {
	case 'n':	// No Parity 
	case 'N':
		options->c_cflag &= ~PARENB;	/* Clear parity enable */
		options->c_cflag &= ~CSTOPB;
	break;
	case 'e':	// Even Parity
	case 'E':
		options->c_cflag |= PARENB;	/* Enable parity */
		options->c_cflag &= ~CSTOPB;	/* 1 stop bit */
		options->c_cflag &= ~PARODD;	/* change to even parity */
		options->c_iflag |= INPCK; 	//XXX input parity check
//		options->c_iflag |= (INPCK | ISTRIP); //XXX input parity check
//		options->c_iflag |= (IGNPAR);	//XXX ignore parity error
//		options->c_iflag |= (IGNBRK);	//XXX ignore break condition,NUL
	break;					/* strip off the 8th bit */
	case 'o':	// Odd Parity
	case 'O':
		options->c_cflag |= PARENB;	 /* Enable parity */
		options->c_cflag &= ~CSTOPB;
		options->c_cflag |= PARODD;	 /* change to odd parity */
		options->c_iflag |= (INPCK | ISTRIP); //XXX input parity check
	break;
	case 'S':	// Space Parity is set the same as no parity
	case 's':	
		options->c_cflag &= ~PARENB;	 /* Clear parity enable */
		options->c_cflag &= ~CSTOPB;
	break;
	default:
		options->c_cflag &= ~PARENB;	 /* Clear parity enable */
		options->c_cflag &= ~CSTOPB;
	}
}

static u_int inline
int2baudrate(int rate)
{
	switch (rate) {
	//B1200;
	//B1800;
	case 2400:
		return B2400;
	break;
	case 4800:
		return B4800;
	break;
	case 9600:
		return B9600;
	break;
	case 19200:
		return B19200;
	break;
	case 38400:
		return B38400;
	break;
	case 57600:
		return B57600;
	break;
	case 115200:
		return B115200;
	break;
	default:
		return B9600;
	}
}

static u_int inline
int2databits(int bits)
{
	switch (bits) {
	case 5:
		return CS5;
	break;
	case 6:
		return CS6;
	break;
	case 7:
		return CS7;
	break;
	case 8:
		return CS8;
	break;
	default:
		return CS8;
	}
}
