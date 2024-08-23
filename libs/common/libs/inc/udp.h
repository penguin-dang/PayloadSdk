#ifndef UDP_HELPER_H_
#define UDP_HELPER_H_

#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <strings.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>

class UDP
{
public:
	bool udp_send(const char *data, int len, const char* host, unsigned port);
};

#endif
