/*
 * Non-Canonical Input Processing,i.e Raw Mode
 * uart.h
 * Serial device read/write method.
 */
#ifndef _UART_H_
#define _UART_H_

#define DEFAULT_BAUDRATE 9600

#include <sys/types.h>

struct uart_config {
	u_int	baudrate;
	u_char	data;	//5,6,7,8,
	u_char	parity;	//none, even(PARENB), odd(PARODD)
	u_char	stop;	//stop bits: 1,2
	u_char	rtscts;	//output HW flow ctrl (0,1 / no,yes)
};

struct uart_port {
	struct uart_config		*uc;
	char				*path;
	int				 fd;
	u_char				 id; //link addr
};


// Only B2400/B4800/B9600 is available
extern int uart_set (const char *path, int *fdp, struct uart_config *uc);
extern int uart_open(const char *path, int *fdp, struct uart_config *uc);
//////////////////////////////////////////////////////////////////////////
extern int uart_close(int *fdp);
extern int uart_read (int *fdp, char *buf, int len);
extern int uart_write(int *fdp, char *buf, int len);

#endif
// uart.c
