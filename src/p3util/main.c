/* ======================================================================== *
 *  Module ............. p3util
 *  File ............... main.c
 *  Creation Date ...... 24 Feb 2014
 *  Author ............. Brian Zenowich
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2003-2008 Barrett Technology, Inc. <support@barrett.com>
 *                          625 Mount Auburn St
 *                          Cambridge, MA 02138, USA
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY BARRETT TECHNOLOGY, INC AND CONTRIBUTORS
 *  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BARRETT
 *  TECHNOLOGY, INC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  The views and conclusions contained in the software and documentation
 *  are those of the authors and should not be interpreted as representing
 *  official policies, either expressed or implied, of Barrett Technology.
 *                                                                          *
 * ======================================================================== */

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/

#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <syslog.h>
#include <sys/mman.h>
#include <unistd.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btos.h"
#include "btcan.h"

int canport;
int startDone = FALSE;
btrt_thread_struct  StartupThread;
struct {int a; char **b;} args;

void handleMenu(int argc, char **argv);

void Startup(void *thd) {
	int argc;
	char **argv;
	int err;

	argc = args.a;
	argv = args.b;

	//printf("a=%d, s=%s\n", argc, argv[1]);

	/* Initialize CAN */
	if(err = initCAN(0, canport)) {
		syslog(LOG_ERR, "initCAN returned err=%d", err);
	}

	if(argc > 1) {
		//getBusStatus(0, status);
		handleMenu(argc, argv);
	} 

	freeCAN(0);
	startDone = TRUE;

	btrt_thread_exit((btrt_thread_struct*)thd);
}

void Cleanup() {
	/* Exit the CANbus thread gracefully */
	StartupThread.done = 1;
	exit(0);
}


int main( int argc, char **argv ) {
	args.a = argc;
	args.b = argv;

	mlockall(MCL_CURRENT | MCL_FUTURE);

	if(fopen("port1", "r")) {
		canport = 1;
	} else {
		canport = 0;
	}

	/* Initialize syslogd */
	openlog("PUCK", LOG_CONS | LOG_NDELAY, LOG_USER);
	syslog(LOG_ERR, "...Starting Puck Utility Program...");

	/* Register the ctrl-c interrupt handler */
	signal(SIGINT, Cleanup);

	/* RT task for setup of CAN Bus */
	btrt_thread_create(&StartupThread,"StTT", 45, (void*)Startup, NULL);
	while(!startDone)
		usleep(10000);
}


void handleMenu(int argc, char **argv) {
	long        status[MAX_NODES];
	int         i, id, cnt = 0;
	char        *c;
	char		slider[8] = {-1};
	char		m[8] = {0};
	char		pot[8] = {0};
	int			value[8];
	long		lval;
	int 		a, key, v;
	FILE		*fp;
					/*GRPA, GRPB, GRPC,   MT,  MOV, HOLD,TSTOP,   KP,   KD, KI, IPNM, POLES,  IKP, IKI, IKCOR */
	int			prop[] = {  26,   27,   28,   43,   47,   77,   78,   79,   80, 81,   86,    90,   91,  92,    93,  -1};
	int			def1[] = {   0,    1,    4, 4500,   37,    0,    0,  500,25000,  0, 1456,    16, 1000, 500,   3000}; /* MF95 */
	int			def2[] = {   0,    1,    4, 1000,   37,    0,    0,  500,25000,  0,  500,    12, 1000, 500,   500}; /* 4DOF */
	int			def3[] = {   0,    1,    4, 1000,   37,    0,    0,  200,16000,  0,  500,     8, 1000, 500,   500}; /* RSF-5B */
	int			*def;
	int			newID, role;
	long		sum;

	c = argv[1];
	while(*c == '-')
		c++;
	*c = toupper(*c);
	printf("\nFunction = %c\n", *c);

	printf("Enumerating...\n");
	getBusStatus(0, status);
	for(i = 0; i < MAX_NODES; i++){
		if(status[i] != -1){
			++cnt;
			id = i;
		}
	}
	printf("Found %d puck%s.\n", cnt, cnt > 1 ? "s" : "");
	
	switch(*c) {
	case 'E': // Enumerate
		for(i = 0; i < MAX_NODES; i++){
			if(status[i] != -1){
				printf("ID %d\n", i);
			}
		}
		printf("Done.\n");
		break;
	case 'P': // Set default properties
		id = atol(argv[2]);
		if(argc > 3)
			a = atol(argv[3]);
		else
			a = 8;
		
		switch(a){
			case 16:
				def = def1;
			break;
			case 12:
				def = def2;
			break;
			case 8:
				def = def3;
			break;
			default:
				printf("\nUsage: ./p3util -p <id> <poles>\n");
				exit(1);
		}
		
		i = 0;
		while(prop[i] != -1){
			printf("Setting property %d to %d...", prop[i], def[i]);
			setProperty(0, id, prop[i], FALSE, def[i]); usleep(1e4);
			setProperty(0, id, SAVE, FALSE, prop[i]); usleep(1e4);
			getProperty(0, id, prop[i], &lval);
			if(lval == def[i]){
				printf("ok.\n");
				
			}else{
				printf("!!! FAIL !!!\n");
				//exit(1);
			}
			++i;
		}
		printf("If POLES was changed, you must cycle power now...\n");
		break;
	case 'S': // Set Serial Number
		id = atol(argv[2]);
		i = atol(argv[3]);
		printf("Setting serial number of puck ID %d to %d...", id, i);
		setProperty(0, id, 2, FALSE, i); usleep(1e4);
		setProperty(0, id, SAVE, FALSE, 2); usleep(1e4);
		getProperty(0, id, 2, &lval);
		if(lval == i){
			printf("ok.\n");
		}else{
			printf("!!! FAIL !!!\n");
			//exit(1);
		}
		break;
	case 'G': // Get properties
		id = atol(argv[2]);
		printf("Getting properties of Puck ID %d\n", id);
		i = 0;
		while(prop[i] != -1){
			getProperty(0, id, prop[i], &lval);
			printf("Property %d = %d\n", prop[i], (int)lval);
			++i;
		}
		break;
	case 'I': // Set ID (& ROLE)
		printf("\n\nChange puck ID: ");
		  if(argc >= 3){
			 id = atol(argv[2]);
			 printf("%d", id);
		  }else{
			 scanf("%d", &id);
		  }

		  if(argc >= 5){
			 newID = atol(argv[4]);
			 printf("\nNew ID: %d", newID);
		  }else{
			 printf("\nNew ID: ");
			 scanf("%d", &newID);
		  }

		  role = -1;
		  if(argc >= 7){
			 role = atol(argv[6]);
			 printf("\nSet ROLE: %d", role);
		  }
		  
		  /* Set the ROLE */
		   if(role >= 0){
			  setProperty(0, id, ROLE, FALSE, role); usleep(1e4);
			  setProperty(0, id, SAVE, FALSE, ROLE); usleep(1e4);
			  getProperty(0, id, VERS, &lval);
		   }
		   setProperty(0, id, ID, 0, newID); usleep(1e4); /* Set the new ID */
		   setProperty(0, newID, SAVE, 0, ID); usleep(1e4); /* Save the new values to EEPROM */
		   getProperty(0, newID, VERS, &lval);
		   
		printf("\nDone. (New ID is active now)\n");
		break;
	case 'F': // Find offsets
		id = atol(argv[2]);
		printf("Finding offsets of Puck ID %d\n", id);
		getProperty(0, id, IOFST, &lval);
		printf("Old IOFST = %d\n", lval);
		sum = 0;
		for(i = 0; i < 32; i++){
			//setProperty(0, id, FIND, FALSE, IOFST);
			//getProperty(0, id, IOFST, &lval);
			getProperty(0, id, IMOTOR, &lval);
			sum += lval;
		}
		sum >>= 5; // /32
		if(sum < 1500 || sum > 2100){
			printf("FAIL: IOFST = %ld\n", sum);
			exit(1);
		}
		printf("New IOFST = %ld\n", sum);
		setProperty(0, id, IOFST, FALSE, sum); usleep(1e4);
		setProperty(0, id, SAVE, FALSE, IOFST); usleep(1e4);
		
		getProperty(0, id, MOFST, &lval);
		printf("Old MOFST = %ld\n", lval);
		
		setProperty(0, id, FIND, FALSE, MOFST); usleep(1e4);
		usleep(6 * 1e6);
		getProperty(0, id, MOFST, &lval);
		printf("New MOFST = %ld\n", lval);
		printf("Done.\n");
		
		break;
	case 'T': // Tune using nanoKontrol2
		if(!cnt){
			// No pucks were found
			printf("\nNo Pucks Found!\n");
			exit(0);
		}
		if(cnt == 1){
			; // We got the ID after the enumeration
		}else if(cnt > 1 && argc >=3){
			// The ID was supplied at the command line
			id = atol(argv[2]);
		}else{
			// Prompt for the ID
			printf("\nEnter Puck ID: ");
			scanf("%d", &id);
		}
		printf("\nTuning gains of puck ID %d\n", id);
		
		// Read the initial values
		printf("Reading initial values...\n");
		getProperty(0, id, V, &lval); value[0] = lval;
		getProperty(0, id, KP, &lval); value[1] = lval;
		printf("Initial KP = %d (%0.4lf)\n", value[1], value[1]/32768.0);
		getProperty(0, id, KD, &lval); value[2] = lval;
		printf("Initial KD = %d (%0.4lf)\n", value[2], value[2]/32768.0);
		getProperty(0, id, KI, &lval); value[3] = lval;
		printf("Initial KI = %d (%0.4lf)\n", value[3], value[3]/32768.0);
		getProperty(0, id, IKP, &lval); value[4] = lval;
		printf("Initial IKP = %d (%0.4lf)\n", value[4], value[4]/32768.0);
		getProperty(0, id, IKI, &lval); value[5] = lval;
		printf("Initial IKI = %d (%0.4lf)\n", value[5], value[5]/32768.0);
		
		printf("Opening NanoKontrol2 device...\n");
		if((fp = fopen("/dev/snd/midiC1D0","r")) == NULL){
			printf("Unable to open NanoKontrol2 device!\n");
			return(1);
		}
	
		printf("Ready to tune!\n");
		while(1){
			a = fgetc(fp); // "176"
			key = fgetc(fp); // id
			v = fgetc(fp); // value
			
			if(key >= 0 && key <= 7){ // Sliders
				if(!m[key]){
					if(slider[key] != -1)
						value[key] += (v-slider[key]) * pow(2,pot[key]/4);
					
					switch(key){
					case 0: // Velocity
						printf("Set V to %d\n", value[key]);
						setProperty(0, id, V, FALSE, value[key]);
						break;
					case 1: // KP
						printf("Set KP to %d\n", value[key]);
						setProperty(0, id, KP, FALSE, value[key]);
						break;
					case 2: // KD
						printf("Set KD to %d\n", value[key]);
						setProperty(0, id, KD, FALSE, value[key]);
						break;
					case 3: // KI
						printf("Set KI to %d\n", value[key]);
						setProperty(0, id, KI, FALSE, value[key]);
						break;
					case 4: // IKP
						printf("Set IKP to %d\n", value[key]);
						setProperty(0, id, IKP, FALSE, value[key]);
						break;
					case 5: // IKI
						printf("Set IKI to %d\n", value[key]);
						setProperty(0, id, IKI, FALSE, value[key]);
						break;
					case 6: // MOV
						printf("Set MOV to %d\n", value[key]);
						setProperty(0, id, MOV, FALSE, value[key]);
						break;
					case 7: // Torque
						printf("Set Torque to %d\n", value[key]);
						setProperty(0, id, T, FALSE, value[key]);
						break;
					}
					//getProperty(0, id, STAT, &lval);
					
				}
				slider[key] = v; // Remember the slider value
			}
			
			if(key >= 16 && key <= 23){ // Pots
				// Record the pot value
				pot[key-16] = v;
			}
			
			if(key == 42){ // Stop
				if(v){
					printf("Stop\n");
					setProperty(0, id, MODE, FALSE, 0);
				}
			}
			
			if(key == 41){ // Run
				if(v){
					printf("Velocity\n");
					setProperty(0, id, TSTOP, FALSE, 0); usleep(1e4);
					setProperty(0, id, MV, FALSE, 32000); usleep(1e4);
					setProperty(0, id, MODE, FALSE, 4); usleep(1e4);
				}
			}
			
			if(key == 45){ // Torque
				if(v){
					printf("Torque\n");
					setProperty(0, id, TSTOP, FALSE, 0); usleep(1e4);
					//setProperty(0, id, MV, FALSE, 32000);
					setProperty(0, id, MODE, FALSE, 2); usleep(1e4);
				}
			}
			
			if(key >= 48 && key <= 55){ // M
				m[key-48] = v;
			}
			
			if(key >= 64 && key <= 71){ // R
				// Reset the value
				value[key-64] = v = 0;
				switch(key-64){
				case 0: // Velocity
					printf("Set V to %d\n", v);
					setProperty(0, id, V, FALSE, v);
					break;
				case 1: // KP
					printf("Set KP to %d\n", v);
					setProperty(0, id, KP, FALSE, v);
					break;
				case 2: // KD
					printf("Set KD to %d\n", v);
					setProperty(0, id, KD, FALSE, v);
					break;
				case 3: // KI
					printf("Set KI to %d\n", v);
					setProperty(0, id, KI, FALSE, v);
					break;
				case 4: // IKP
					printf("Set IKP to %d\n", v);
					setProperty(0, id, IKP, FALSE, v);
					break;
				case 5: // IKI
					printf("Set IKI to %d\n", v);
					setProperty(0, id, IKI, FALSE, v);
					break;
				case 7: // Torque
					printf("Set T to %d\n", v);
					setProperty(0, id, T, FALSE, v);
					break;
				}
				//getProperty(0, id, STAT, &lval);
			}
			
			if(key >= 33 && key <= 37 && v == 0){ // S
				// Save the value to EEPROM
				switch(key-32){
				case 0: // Velocity
					printf("Saved V\n");
					setProperty(0, id, SAVE, FALSE, V);
					break;
				case 1: // KP
					printf("Saved KP\n");
					setProperty(0, id, SAVE, FALSE, KP);
					break;
				case 2: // KD
					printf("Saved KD\n");
					setProperty(0, id, SAVE, FALSE, KD);
					break;
				case 3: // KI
					printf("Saved KI\n");
					setProperty(0, id, SAVE, FALSE, KI);
					break;
				case 4: // IKP
					printf("Saved IKP\n");
					setProperty(0, id, SAVE, FALSE, IKP);
					break;
				case 5: // IKI
					printf("Saved IKI\n");
					setProperty(0, id, SAVE, FALSE, IKI);
					break;
				}
				//getProperty(0, id, STAT, &lval);

			}
			
			//usleep(10000);
		}
		
		break;
	default: // Print help text
		printf("\nUsage:\n./p3util [e p i g f t]\ne = Enumerate\np = Set default properties\ni = Set ID (& ROLE)\ng = Get properties\nf = Find offsets\nt = Tune motor\n");
		break;
		
	}
	printf("\n\n");
}


