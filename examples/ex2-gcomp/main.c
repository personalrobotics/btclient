/*======================================================================*
 *  Module .............Example 1 - WAM Position
 *  File ...............main.c
 *  Author .............Traveler Hauptman
 *  Creation Date ......26 May 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 * 
 *                                                                      *
 *======================================================================*/

/** \file main.c
    A minimalist program for the wam that prints out the position of the 
    end point of the WAM
 
 */


#include <syslog.h>
#include <rtai_lxrt.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include "btwam.h"

btthread wam_thd;
wam_struct *wam;

void sigint_handler()
{
    btthread_stop(&wam_thd); // Stop the WAMControlThread
    CloseWAM(wam); // Free the wam data structure
    printf("\n\n");
    exit(1);
}

int main(int argc, char **argv)
{
    char buf[80];
    int  useGimbals = 0;
    int  err, i;
    char robotName[128];
    
    /* Initialize syslog */
    openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
    atexit((void*)closelog);

#ifndef BTOLDCONFIG

    err = ReadSystemFromConfig("wam.conf");
#else //BTOLDCONFIG
#endif //BTOLDCONFIG
    /* If the robot name was given on the command line, use it */
    *robotName = 0;
    for(i = 1; i < argc-1; i++) {
        if(!strcmp(argv[i],"-n"))
            strcpy(robotName, argv[i+1]);
    }

    /* Initialize and get a handle to the robot */
    if(!(wam = OpenWAM("wam.conf", robotName)))
        exit(1);

    /* Check and handle any additional command line arguments */
    for(i = 1; i < argc-1; i++) {
        if(!strcmp(argv[i],"-g")) // If gimbals are being used
        {
            initGimbals(wam);
            useGimbals = TRUE;
            syslog(LOG_ERR, "Gimbals expected.");
        }
    }

    /* Register the ctrl-c interrupt handler */
    signal(SIGINT, sigint_handler);

    /* Start up wam control loop */
    wam_thd.period = 0.002;
    btthread_create(&wam_thd,90,(void*)WAMControlThread,(void*)wam);

    printf("\nActivate the WAM, then press <Enter>");
    fflush(stdout);
    fgetc(stdin);
    SetGravityComp(wam, 1.0); // Set gravity scale to 1.0g
    
    printf("\nPress Ctrl-C to exit...\n");
    while(1) {
        printf("\rJoint Torque = %s\t",sprint_vn(buf,(vect_n*)wam->Jtrq));
        fflush(stdout);
        usleep(100000); // Sleep for 0.1s
    }

    btthread_stop(&wam_thd); //Kill WAMControlThread
    CloseWAM(wam);
    exit(1);
}




