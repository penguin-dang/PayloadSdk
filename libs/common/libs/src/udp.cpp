#include "udp.h"

// /**
//  * @fn udp_send
//  * @summary quick and dirty C++ function to send a udp packet to an address
//  * @example udp_send("hello", 6 /* strlen("hello")+1 */, "127.0.0.1", 9898)
//  */

bool UDP::udp_send(const char *data, int len, const char* host, unsigned port)
{
	sockaddr_in servaddr;

	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(fd < 0)
		return false; // failed to open a socket, permission issues?

	bzero(&servaddr,sizeof(servaddr));
	servaddr.sin_family = AF_INET;

	if (inet_aton(host, &servaddr.sin_addr)== 0)
		return close(fd), false; // failed to parse the host address

	servaddr.sin_port = htons(port);

	if (sendto(fd, data, len, 0, (sockaddr*)&servaddr, sizeof(servaddr)) < 0)
		return close(fd), false; // send failed

	return close(fd), true;
}
