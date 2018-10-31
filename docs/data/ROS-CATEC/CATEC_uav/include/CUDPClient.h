/*
 * CUDPClient.h
 *
 *  Created on: Jan 25, 2012
 *      Author: avionica
 */

#ifndef CUDPCLIENT_H_
#define CUDPCLIENT_H_

#include<sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <cstring>

/*!
 * \brief Used to connect to a UDP server and send information.
 */

class CUDPClient{
public:
	/*!
	 * \brief Default constructor.
	 */
	CUDPClient()
	{
		s		=	-1;
		slen	=	sizeof(si_other);
	}
	/*!
	 * \brief Connects to an UDP server.
	 * \param srv_ip The server IP Address, for example "10.0.0.40".
	 * \param port The server port, for example 9930.
	 */
	void connect(const char* srv_ip, unsigned int port)
	{
		if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
		{
			fprintf(stderr, "socket failed\n");
			return;
		}

		  memset((char *) &si_other, 0, sizeof(si_other));
		  si_other.sin_family = AF_INET;
		  si_other.sin_port = htons(port);
		  if (inet_aton(srv_ip, &si_other.sin_addr)==0) {
			  fprintf(stderr, "inet_aton() failed\n");
			  return;
		  }
	}
	/*!
	 * \brief Sends a message to the UDP server.
	 * \param cad Buffer containing the data to be sent.
	 * \param size Size of the buffer in bytes.
	 * \return 1 if the message is sent and 0 if we are not connected to the server.
	 */
	bool send(const void* cad, size_t size)
	{
		if (s != -1)
		{
			sendto(s, cad, size, 0,(sockaddr*)&si_other, slen);
			return true;
		}
		return false;
	}
	/*!
	 * \brief Disconnects from the server.
	 */
	void disconnect()
	{
		close(s);
	}
	/*!
	 * \brief Destructor.
	 */
	~CUDPClient()
	{
		disconnect();
	}

private:


	struct sockaddr_in si_other;
	int s, slen;

};


#endif /* CUDPCLIENT_H_ */
