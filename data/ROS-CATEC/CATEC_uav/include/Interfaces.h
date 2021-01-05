/*
 * Interfaces.h
 *
 *  Created on: 13/01/2012
 *      Author: CATEC
 */

#ifndef INTERFACES_H_
#define INTERFACES_H_


/*#include <net/netbyte.h>
#include <sys/iomsg.h>
#include <sys/neutrino.h>
#include "ar_err.h"
#include "ar_err_codes.h"
#include "ar_shm_types.h"
#include "ar_shm_int.h"
#include "ar_config.h"*/
#include <CATEC_interfaces_types.h>

#ifndef SRV_IP
#define SRV_IP "10.0.0.40"
#endif

using namespace std;


typedef struct {
//	process_base_config_t	base;

	int shm_IControlRefRw_OUT1;
	int shm_IControlRefRw_OUT2;
	int shm_IControlRefRw_OUT3;
	int shm_IControlRefRw_OUT4;
	int shm_IControlRefRw_OUT5;
	int shm_IControlRefRw_OUT6;
	int shm_IControlRefRw_OUT7;
	int shm_IControlRefRw_OUT8;
	int shm_IControlRefRw_OUT9;
	int shm_IControlRefRw_OUT10;

	int shm_ICommandFlagsRw_OUT1;
	int shm_ICommandFlagsRw_OUT2;
	int shm_ICommandFlagsRw_OUT3;
	int shm_ICommandFlagsRw_OUT4;
	int shm_ICommandFlagsRw_OUT5;
	int shm_ICommandFlagsRw_OUT6;
	int shm_ICommandFlagsRw_OUT7;
	int shm_ICommandFlagsRw_OUT8;
	int shm_ICommandFlagsRw_OUT9;
	int shm_ICommandFlagsRw_OUT10;
	int port;

} template_config_t;

typedef struct {
	int struct_type;
	int uav_id;
	union
	{
		IControlRefRw control_ref;
		ICmdFlagsRw   command_flags;
	};
} ROS_control_datagram;


#endif /* INTERFACES_H_ */
