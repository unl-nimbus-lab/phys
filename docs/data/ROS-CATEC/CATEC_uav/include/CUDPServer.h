/*
 * CUDPServer.h
 *
 *  Created on: Jan 25, 2012
 *      Author: avionica
 */

#ifndef CUDPSERVER_H_
#define CUDPSERVER_H_

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <cstring>

/*!
 * \brief Used to create UDP server and receive information.
 */

class CUDPServer{
public:
	int s, slen;
	/*!
	 * \brief Default constructor.
	 */
	CUDPServer()
	{
		s		=	-1;
		slen	=	sizeof(si_other);
	}
	/*!
	 * \brief Creates a UDP server.
	 * \param port The server port, for example 9930.
	 */
	void connect(unsigned int port)
	{
		    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
			{
				fprintf(stderr, "socket failed\n");
				return;
			}

		    memset((char *) &si_me, 0, sizeof(si_me));
		    si_me.sin_family = AF_INET;
		    si_me.sin_port = htons(port);
		    si_me.sin_addr.s_addr = htonl(INADDR_ANY);

		    if (bind(s, (const sockaddr*)&si_me, sizeof(si_me))==-1)
			{
		    	s = -1;
				fprintf(stderr, "bind failed\n");
				return;
			}
	}
	/*!
	 * \brief Receives a message from a client.
	 * \param buf Buffer to be filled.
	 * \param len Length of the buffer in bytes.
	 * \param from Points to a sockaddr structure in which the sending address is to be stored
	 * \param fromlen Specifies the length of the sockaddr structure pointed to by the from argument.
	 * \return 1 if the message is sent and 0 if we have not created server.
	 */
	ssize_t receive(void* buf, size_t len, struct sockaddr* from, socklen_t* fromlen)
	{
		if(s!=-1)
			return recvfrom(s, buf, len, 0, from, fromlen);
		else
			return -1;
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
	~CUDPServer()
	{
		disconnect();
	}

private:

	struct sockaddr_in si_me, si_other;


};


#endif /* CUDPSERVER_H_ */
