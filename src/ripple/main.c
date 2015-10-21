/* ======================================================================== *
 *  Module ............. Example 1 - Joint Position
 *  File ............... main.c
 *  Creation Date ...... 26 May 2005
 *  Author ............. Traveler Hauptman
 *  Addtl Authors ...... Brian Zenowich
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2005-2008 Barrett Technology, Inc. <support@barrett.com>
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

/** \file main.c
    A minimalist program for the WAM that prints out the position of the 
    end point of the WAM
 */

#define toupper(c)      ( ((c >= 'a') && (c <= 'z')) ? c - ('a' - 'A') : c )
#define HOLE            (-1)

#define TEST_TR         (0)
#define COLLECT_TR      (1)
#define TR_DATA	        (2)

#ifndef TRUE
#define TRUE (1)
#endif
#ifndef FALSE
#define FALSE (0)
#endif

#define NULL_VALUE (-9999)

/* Provide access to standard C input/output functions such as printf() */
#include <stdio.h>

/* Provides atexit(), registers functions to execute upon process termination */
#include <stdlib.h>

/* The syslog daemon is used primarily for sending debug information to a log 
 * file. Note that syslog calls break realtime scheduling, so try not to use
 * syslog() from within a realtime thread!
 */
#include <syslog.h>

/* Allow us to catch the Ctrl-C signal and exit gracefully */
#include <signal.h>

/* Provides mlockall(), prevent process memory from being swapped out to disk */
#include <sys/mman.h>

#include <fcntl.h>

/* Include the standard WAM header file */
#include "btcan.h"
#include "btcontrol.h"

int     puckID;
int     motorSN;
long    encCtsPerRev;
int     fwd;
int     rev;
long    samplesPerRev;
int     smoothCt;
int     epsilon;
int     waitSamples;
btPID   p;
int     CANdev;
int     progMode;
double  sampleTime;
int     scalePct;
char 	**arg;

/* Define the realtime threads */
btrt_thread_struct    rt_thd, puck_thd, main_thd;

/* Flag to let the main() thread know when initialization is complete */
int startDone;

int compTorque = 0;
int aTf[4096], aTr[4096], aCts[4096];

/* Function prototypes */
void Cleanup();
void sigint_handler();
void rt_thread(void *thd);

/* If Ctrl-C is pressed, exit gracefully */
void sigint_handler()
{
   Cleanup();
   exit(1);
}

/* For Debugging: This function will execute when a RealTime thread switches to 
 * secondary mode.  Also, one can look at /proc/xenomai/stat and MSW will 
 * tell you the number of switches a thread has made.
 */
void warn_upon_switch(int sig __attribute__((unused)))
{
    void *bt[32];
    int nentries;
    int fd;
 
     // Open backtrace file as APPEND. Create new file if necessary (chmod 644)
    fd = open("sigxcpu.txt", O_WRONLY | O_APPEND | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    
    /* Dump a backtrace of the frame which caused the switch to secondary mode
       To decipher a backtrace:
       1) Look at the line of the backtrace originating in your application (not a library,
          and not the warn_upon_switch function)
       2) Take note of the memory address of that line, ex: [0x80588d5]
       3) Run "gdb <progname>"
       4) Type "list *0x80588d5" (substitute your actual memory address)
       5) gdb will show the function() (file:line) information for your error, followed by 
          a listing of several surrounding lines
    */
    nentries = backtrace(bt, sizeof(bt) / sizeof(bt[0]));
    backtrace_symbols_fd(bt, nentries, fd);
    write(fd, "-----\n", 6); // Output a separator line
    
    close(fd);
    syslog(LOG_ERR, "Switched out of RealTime, see sigxcpu.txt for details (frame backtrace)");
}

void main_thread(void *thd){
	char **argv = arg;
	   FILE* 	fp;
   int	 	data;
   long 	reply;
   int 		puckID;
   int		lineCt;
   int		progress = 0;
   int		i;
	
	if(progMode == TR_DATA){
		// [1] = -d, [2] = id, [3] = -f, [4] = file
		 if((fp = fopen(argv[4],"r")) == NULL){
			 printf("Could not open %s, exiting.", argv[2]);
			 exit(1);
		 }
		 fscanf(fp, "%*s"); // Skip first line
		 puckID = atoi(argv[2]);
		 fscanf(fp, "%d", &lineCt); // Line count
		 printf("Data download to puck ID: %d\n", puckID);
		 CANdev = 0;
		 for(i = 0; i < lineCt; i++){
			 fscanf(fp, "%d", &data); // Read data
			 setProperty(CANdev, puckID, ADDR, FALSE, -256-i*2); // Set address
			 setProperty(CANdev, puckID, VALUE, FALSE, data); // Write data
			 getProperty(CANdev, puckID, MECH, &reply); // Ensure puck is ready
			 progress = i * 100.0 / lineCt; 
			 printf("Progress: %d\r", progress);
			 fflush(stdout);
		 }
		 printf("\nDownload complete.\n");
		 fclose(fp);
		 
		 exit(0);
	}
	
   doTR(progMode);
}

/* The CANbus card must be initialized and called from a realtime thread.
 * The rt_thread is spun off from main() to handle the initial communications.
 */
void rt_thread(void *thd){
   int err;
   long status[MAX_NODES];
   
   /* Initialize CAN */
   if(err = initCAN(0, 0)) {
      printf("initCAN returned err=%d", err);
      exit(1);
   }
   
   getBusStatus(0, status);
   
   /* Notify main() thread that the initialization is complete */
   startDone = TRUE;
   
   /* Spin until we are told to exit */
   while (!btrt_thread_done((btrt_thread_struct*)thd)){
      usleep(10000);
   }
   
   /* Remove this thread from the realtime scheduler */
   btrt_thread_exit((btrt_thread_struct*)thd);
}

/* Exit the realtime threads and close the system */
void Cleanup(){
   /* Tell the WAM control thread to exit */
   puck_thd.done = TRUE;
   
   /* Wait for the control thread to exit, then free any data and device locks
    * associated with the WAM. The wait is so that we do not free the device
    * while the control loop is still using it!
    */
   usleep(10000);
   
   setProperty(CANdev, puckID, MODE, FALSE, MODE_IDLE);
   CloseSystem();
   
   /* Tell the initial communcation thread to exit */
   rt_thd.done = TRUE;
   
   /* Put some distance between the last printed data and the user's prompt */
   printf("\n\n");
}

void PuckControlThread(void *data)
{
   btrt_thread_struct* this_thd;
   double            thisperiod;
   RTIME             rtime_period, sampleCount;
   RT_TASK           *WAMControlThreadTask;
   long               actPos;
   
   /* Get the wam pointer */
   this_thd = (btrt_thread_struct*)data;
   
   /* Set up timer */
   thisperiod = this_thd->period;
   rtime_period = (RTIME)(thisperiod * 1000000000.0);

#ifdef RTAI

   sampleCount = nano2count(rtime_period);
   rt_set_periodic_mode();
   start_rt_timer(sampleCount);

   WAMControlThreadTask = rt_task_init(nam2num(this_thd->name), 5, 0, 0);
   rt_task_make_periodic_relative_ns(WAMControlThreadTask, rtime_period, rtime_period);

#endif

   syslog(LOG_ERR,"WAMControl period Sec:%f, ns: %d", thisperiod, rtime_period);

   /*Set up as hard real time*/
   btrt_set_mode_hard();

   /* Install a signal handler to log when the thread is changing over to secondary mode.
      Almost any I/O in a realtime thread will cause the thread to be pulled out
      of the realtime domain and placed in the domain of the standard Linux scheduler.
      This includes printf(), syslog(), send(), receive(), read(), write(), and many
      other system calls. To keep the control loop in the realtime domain, you should
      avoid calling these functions from a realtime thread. Excessive latencies, 
      heartbeat errors, and unstable control can all result from the control loop
      being forced out of primary mode (realtime domain) by a realtime-unfriendly
      function call!
    */
#ifdef XENOMAI
   signal(SIGXCPU, warn_upon_switch); // Catch the SIGXCPU signal
   btrt_set_mode_warn(); // Enable the warning
#endif

#ifdef XENOMAI
   /*Make task periodic*/
   test_and_log(
      rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks((rtime_period))),"WAMControlThread: rt_task_set_periodic failed, code");
#endif

   while (!btrt_thread_done(this_thd)) {

      /*Make sure task is still in Primary mode*/
      btrt_set_mode_hard();
      
      /* Wait until the start of the next control period */
      btrt_task_wait_period();
      
      /* Clear CAN bus of any unwanted messages */
      canClearMsg(CANdev);

      /* Read the motor positions from the robot */
      getProperty(CANdev, puckID, P, &actPos);
      
      /* Update PID */
      sety_btPID(&p, actPos);
      step_btPID(&p);
      
      /* Set the new torque */
      setProperty(CANdev, puckID, T, FALSE, (int)(p.lastresult) + compTorque);

   }
#ifdef XENOMAI
   rt_task_set_mode(T_WARNSW, 0, NULL); // Disable mode switch warning
#endif
   syslog(LOG_ERR, "WAM Control Thread: exiting");
   
   /*Delete thread when done*/
   btrt_thread_exit(this_thd);
}

int doTR(int doCollect)
{
    int         err;
    FILE        *in;
    long        actPos;
   
   init_btPID(&p);
   
    if((in = fopen("tr.info","r")) == NULL)
    {
        printf("\nFailed to open tr.info file!");
        return(1);
    }
    fscanf(in, "%*s : %d", &puckID);
    fscanf(in, "%*s : %d", &motorSN);
    fscanf(in, "%*s : %ld", &encCtsPerRev);
    fscanf(in, "%*s : %d %*s : %d %*s : %ld %*s : %d %*s : %d %*s : %d", &fwd, &rev, &samplesPerRev, &smoothCt, &epsilon, &waitSamples);
    fscanf(in, "%*s : %lf %*s : %lf %*s : %lf", &p.Kp, &p.Kd, &p.Ki);
    fscanf(in, "%*s : %d %*s : %lf", &scalePct, &sampleTime);
    
    fclose(in);
    
    printf( "CANID      : %d\n"
            "MOTOR_SN   : %d\n"
            "CTS_PER_REV: %d\n"
            "FWD_REVS   : %d \n"
            "REV_REVS   : %d \n"
            "SAMP/REV   : %ld \n"
            "SMOOTH_CT  : %d \n"
            "STOPPED e  : %d \n"
            "STOPPED CT : %d \n"
            "KP         : %0.3f \n"
            "KD         : %0.3f \n"
            "KI         : %0.3f\n"
            "SCALE_PCT  : %d \n"
            "SAMPLE_TIME: %0.5f \n", puckID, motorSN, encCtsPerRev,
            fwd, rev, samplesPerRev, smoothCt, epsilon, waitSamples,
            p.Kp, p.Kd, p.Ki, scalePct, sampleTime);

    CANdev = 0;
    
    printf("About to initTR()\n");
    initTR();
    
    compTorque = 0;
    printf("About to get P\n");
    getProperty(CANdev, puckID, P, &actPos);
    printf("Present position = %ld\n", actPos);
    

    //setyref_btPID(&p, actPos);
    
    printf("Starting control thread\n");
    puck_thd.period = sampleTime; // Control loop period in seconds
        
   setgains_btPID(&p, p.Kp, p.Kd, p.Ki);
   setinputs_btPID(&p, actPos, actPos, puck_thd.period);
    btrt_thread_create(&puck_thd, "puck", 90, PuckControlThread, (void*)NULL);
    //start_control_threads( 40, 1, PuckControlThread, (void*)&CThreadParams);
    
    usleep(1000000);
    
    /* Collect the data */
    if(doCollect)
        err = collectTR();
    
    err = testTR();
    
    return(err);
}

void motorSettle()
{
    long reply, lastReply;
    int ctsRemaining;
    
    //getProperty(CANdev, puckID, P, &lastReply);
    //usleep(1000);
    
    ctsRemaining = waitSamples;
    lastReply = p.y;
    while(ctsRemaining)
    {
       //printf("\rp.y=[%8ld], last=[%8ld], ctsrem=[%8d]", (long)p.y, lastReply, ctsRemaining);
       //fflush(stdout);
        //getProperty(CANdev, puckID, P, &reply);
        if(abs((long)p.y - lastReply) <= epsilon) // If we are within episilon
        {
            --ctsRemaining;
        }else{
            ctsRemaining = waitSamples;
        }
        
        lastReply = p.y;  
        usleep(1/sampleTime);  
    }
    //printf("\n");
}

/* The collected torque array may have gaps (NULL_VALUE).
 * Fill the gaps with linearly interpolated data.
 * Alters array in-place.
 */
void fillHoles(int *aTrq, long ctsPerRev)
{
    int 	i, j, k;
    double  inc, sum;
    
    i = 0;
    while(1)
    {
        if(aTrq[i] != NULL_VALUE){
        	++i;
        	if(i >= ctsPerRev)
        		return;
        	continue;
        }
        
        // Else, we hit a NULL_VALUE
        // Find the prev non-NULL
        j = i;
        while(1){
        	--j;
        	if(j < 0) 
        		j += ctsPerRev;
        	if(aTrq[j] != NULL_VALUE)
        		break;
        }
        
        // Find the next non-NULL
        k = i;
        while(1){
        	++k;
        	if(k >= ctsPerRev)
        		k = 0;
        	if(aTrq[k] != NULL_VALUE)
        		break;
        }
        
        // Interpolate (from j to k) to fill in the gap
        if(k == j) // Special case, single non-NULL in array
        	inc = 0.0;
        else if(k > j)
        	inc = (double)(aTrq[k] - aTrq[j]) / (k - j);
        else
        	inc = (double)(aTrq[k] - aTrq[j]) / (k + ctsPerRev - j);
        
        sum = 0.0;
        
        while(1){
        	sum += inc;
        	aTrq[i] = (int)(sum + 0.5); // Round properly
        	++i;
        	if(i >= ctsPerRev)
        		return;
        	if(i == k)
        		break;
        }
    }
}

void smoothData(int *Tc, int *Tf, long ctsPerRev, int smoothCt)
{
    int     i;
    int     torqueSum;
    long    modPos;
    int     count;
    
    /* Average every SMOOTH_CT data points of Tf into Tc */
    for(i = 0; i < ctsPerRev; i++)
    {
        torqueSum = 0;
        for(count = 0; count < smoothCt; count++)
        {
            modPos = i + count - (smoothCt-1) / 2;
            modPos %= ctsPerRev;
            if(modPos < 0) modPos += ctsPerRev;
            torqueSum += Tf[modPos];
            
        }
        Tc[i] = torqueSum / smoothCt;
    }
    
    /* Save the smoothed values back into Tf */
    for(i = 0; i < ctsPerRev; i++)
    {
        Tf[i] = Tc[i];
    }
}

void outputData(FILE *outFile, int *aTrq, long ctsPerRev, char dir, int revolution)
{
    int i;
    
    for(i = 0; i < ctsPerRev; i++)
    {
        fprintf(outFile, "%c\t%d\t%d\n", dir, revolution, aTrq[i]);
    }
}

int collectTR()
{
    long    pOffset;
    FILE    *outFile;
    long    reply;
    int     revolution;
    long    sampleDistance;
    long    modPos;
    int     i;
    char    fileName[20];
    long    torqueSum;
    int     count;
    long 	mech;
    
    sampleDistance = encCtsPerRev / samplesPerRev;
    
    printf("\nCollecting Data...\n");
    
    printf("Allocating TR arrays: %d elements\n", encCtsPerRev);
    #if 0
    if((aTf = (int *)malloc(encCtsPerRev * sizeof(int))) == NULL);
    {
        syslog(LOG_ERR, "Could not allocate aTrq array");
        exit(1);
    }
    
    if((aTr = (int *)malloc(encCtsPerRev * sizeof(int))) == NULL);
    {
        syslog(LOG_ERR, "Could not allocate aTrq array");
        exit(1);
    }
    
    if((aCts = (int *)malloc(encCtsPerRev * sizeof(int))) == NULL);
    {
        syslog(LOG_ERR, "Could not allocate aCts array");
        exit(1);
    }
       #endif 
    /* Write the *.raw file */
    sprintf(fileName, "motor%d.raw", motorSN);
    if((outFile = fopen(fileName, "w")) == NULL)
    {
        syslog(LOG_ERR, "Unable to open '%s' for writing!", fileName);
        return(1);
    }
    fprintf(outFile, "Dir\tRev#\tSamples\tCmdPos\tActPos\tCmdTrq\n");

    // Set the initial position
    printf("Resetting initial position to zero\n");
    setProperty(CANdev, puckID, MODE, FALSE, MODE_IDLE);
    setProperty(CANdev, puckID, P, FALSE, 0L);
    usleep(1/sampleTime);
    setyref_btPID(&p, 0);
    reset_btPID(&p);
    
    // Engage PID controller
    printf("About to set MODE_TORQUE\n");
    start_btPID(&p);
    setProperty(CANdev, puckID, MODE, FALSE, MODE_TORQUE);
    printf("PID controller engaged\n");
    //printf("Starting MECH = %ld\n", mech);
    printf("y=%lf yref=%lf Kp=%lf Kd=%lf dt=%lf\n", 
    p.y,
    p.yref,
    p.Kp,
    p.Kd,
    p.dt);
    
#if 0    
    while(reply > 5)
    {
        setyref_btPID(&p, p.yref + 1);
        usleep(1/sampleTime);
        getProperty(CANdev, puckID, MECH, &reply);
        //printf("\nyref = %f, MECH=%ld", p.yref, reply);
    }
#endif   

    // Display the initial MECH
    getProperty(CANdev, puckID, MECH, &mech);
    printf("Starting MECH = %ld\n", mech);

    // Display the initial P
    printf("Starting P = %ld\n", p.y);
    
    // Calculate the offset
    pOffset = (int)(p.y - mech) % encCtsPerRev; // (p.y - pOffset) % encCtsPerRev = MECH
    printf("Offset = %ld (%5.2f - %ld)\n", pOffset, p.y, mech);
    
    //cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    
    printf("Collecting data\n");
    
    // Clear aTf to zeros
    for(i = 0; i < encCtsPerRev; i++)
        aTf[i] = NULL_VALUE;
        
    for(revolution = 1; revolution <= fwd; revolution++)
    {
        // Clear aCts to zeros
        //for(i = 0; i < encCtsPerRev; i++)
        //    aCts[i] = HOLE;
            
        for(i = 0; i < samplesPerRev; i++)
        {
            /* Go to the new position */
            setyref_btPID(&p, p.yref + sampleDistance);
            
            mech = ((long)p.y - pOffset) % encCtsPerRev; // Get MECH
            if(mech < 0) mech += encCtsPerRev; // Handle negative side
            if(revolution > 1)
            	compTorque = aTf[mech] * scalePct / 100.0;
            else
            	compTorque = 0;
            
            /* Wait for motor position to settle */ 
            motorSettle();
            
            /* Record the torque */
            //aCts[i] = modPos; // Record MECH
            aTf[mech] = (int)p.lastresult + compTorque;
        }
        
        // Fill the holes
        printf("Filling Holes...\n");
        fillHoles(aTf, encCtsPerRev);
        
        // Smooth the data
        printf("Smoothing Data...\n");
        smoothData(aCts, aTf, encCtsPerRev, smoothCt);
        
        // Output the data
        printf("Output data...\n");
        outputData(outFile, aTf, encCtsPerRev, 'F', revolution);
    }
    
    // Clear aTr to zeros
    for(i = 0; i < encCtsPerRev; i++)
        aTr[i] = NULL_VALUE;
        
    for(revolution = 1; revolution <= rev; revolution++)
    {
        // Clear aCts to zeros
        //for(i = 0; i < encCtsPerRev; i++)
        //    aCts[i] = HOLE;
            
        for(i = 0; i < samplesPerRev; i++)
        {
            /* Go to the new position */
            setyref_btPID(&p, p.yref - sampleDistance);
            mech = ((long)p.y - pOffset) % encCtsPerRev;
            if(mech < 0) mech += encCtsPerRev;
            if(revolution > 1)
            	compTorque = aTr[mech] * scalePct / 100.0;
            else
            	compTorque = 0;
            
            /* Wait for motor position to settle */ 
            motorSettle();
            
            /* Record the torque */
            //aCts[i] = modPos;
            aTr[mech] = (int)p.lastresult + compTorque;
        }
        
        // Fill the holes
        fillHoles(aTr, encCtsPerRev);
        
        // Smooth the data
        smoothData(aCts, aTr, encCtsPerRev, smoothCt);
        
        // Output the data
        outputData(outFile, aTr, encCtsPerRev, 'R', revolution);
    }
    
    /* Close the RAW data file */
    fclose(outFile);
    stop_btPID(&p);
    
    /* Smooth it out */
    /* Average every SMOOTH_CT data points of Tf and Tr, then average Tf and Tr into Tc */
    for(i = 0; i < encCtsPerRev; i++)
    {
        torqueSum = 0;
        for(count = 0; count < smoothCt; count++)
        {
            modPos = i + count - (smoothCt-1) / 2;
            modPos %= encCtsPerRev;
            if(modPos < 0) modPos += encCtsPerRev;
            torqueSum += aTf[modPos];
            torqueSum += aTr[modPos];
        }
        aCts[i] = torqueSum / (smoothCt * 2);
    }
    
    /* Write the data to a file */
    sprintf(fileName, "motor%d.tr", motorSN);
    if((outFile = fopen(fileName, "w")) == NULL)
    {
        printf("\nUnable to open '%s' for writing!", fileName);
        return(1);
    }
    fprintf(outFile, "%d\n", encCtsPerRev);
    
    /* Output the data */
    for(i = 0; i < encCtsPerRev; i++)
    {
        /* Save the combined data */
        fprintf(outFile, "%d\n", aCts[i]);
    }
    
    /* Close *.tr data file */
    fclose(outFile);
    
    printf("\nDone collecting TR data!\n");
    
    /* Clean up */
    //free(aTf);
    //free(aTr);
    //free(aCts);
    
    return(0);
}


    
int testTR()
{
    long    pOffset;
    long    reply;
    long    mech;
    int     i;
    int		hysteresis_distance = 5;
    long	hysteresis_value;
    FILE    *in;
    char    fileName[20];
    
    //printf("\nPlease enter *.tr file name: ");
    //scanf("%s", fileName);
    sprintf(fileName, "motorBrs2.trm", motorSN);
    //sprintf(fileName, "motor%d.trm", motorSN);
    //sprintf(fileName, "motor%d.raw", motorSN);
    
    if((in = fopen(fileName, "r")) == NULL)
    {
        printf("\nFailed to open torque ripple file [%s]!\n", fileName);
        return(1);
    }
#if 0
    /* Allocate TorqueRipple Compensation array space */
    aTrq = (int *)malloc(encCtsPerRev * sizeof(int));
    if(aTrq == NULL) /* Failed to allocate buffer */
    {
        printf("\nFailed to allocate aTrq buffer!");
        return(1);
    }
#endif
    //fscanf(in, "%d", &encCtsPerRev); 
    //encCtsPerRev = 4096;
    fscanf(in, "%*s"); // Skip first line
    for(i = 0; i < encCtsPerRev; i++){
    	fscanf(in, "%d", &aTf[i]);
    }
    //for(i = 0; i < encCtsPerRev; i++)
    //{
    //    fscanf(in, "%*c %*d %d", &aTf[i]);
    //}
    for(i = 0; i < encCtsPerRev; i++)
    {
        fscanf(in, "%d", &aTr[i]);
    }
    fclose(in);

	// Set the initial position
    setProperty(CANdev, puckID, MODE, FALSE, MODE_IDLE);
    setProperty(CANdev, puckID, P, FALSE, 0L);
    usleep(1/sampleTime);
    setyref_btPID(&p, 0);
    reset_btPID(&p);
    
    // Engage PID controller
    printf("About to set MODE_TORQUE\n");
    p.Ki = 0;
    p.Kd = 0;
    p.Kp = 0;
    start_btPID(&p);
    setProperty(CANdev, puckID, MODE, FALSE, MODE_TORQUE);
    printf("PID controller engaged\n");
    //printf("Starting MECH = %ld\n", mech);
    printf("y=%lf yref=%lf Kp=%lf Kd=%lf dt=%lf\n", 
    p.y,
    p.yref,
    p.Kp,
    p.Kd,
    p.dt);
 
    // Display the initial MECH
    getProperty(CANdev, puckID, MECH, &mech);
    printf("Starting MECH = %ld\n", mech);

    // Display the initial P
    printf("Starting P = %ld\n", p.y);
    
    // Calculate the offset
    pOffset = (int)(p.y - mech) % encCtsPerRev; // (p.y - pOffset) % encCtsPerRev = MECH
    printf("Offset = %ld (%5.2f - %ld)\n", pOffset, p.y, mech);
    
    printf("\nTR Test, press ctrl-c to exit...\n\n"); fflush(stdout);
    
    hysteresis_value = p.y;
    
    while(1)
    {
        // Update hysteresis_value, if necessary
        if(p.y - hysteresis_value > hysteresis_distance)
        	hysteresis_value = p.y - hysteresis_distance;
        if(p.y - hysteresis_value < -hysteresis_distance)
        	hysteresis_value = p.y + hysteresis_distance;
        
        setyref_btPID(&p, p.y); // This may not be required
        
        // Determine MECH
        mech = ((long)p.y - pOffset) % encCtsPerRev;
        if(mech < 0) mech += encCtsPerRev;
        //compTorque = aTf[mech]*1.5;
        
        // Apply compensation torque in correct direction
        if(p.y > hysteresis_value)
        	compTorque = aTf[mech];
        else if(p.y < hysteresis_value)
        	compTorque = aTr[mech];
        else
        	compTorque = 0;
        
        // Show the compensation torque value
        printf("\rcompTorque = %5d     ", compTorque);
        usleep(1.0/sampleTime);
        
    }
    
}

int initTR()
{
    long reply;
    //int torque;
    char c;
    int i;

    printf("Waking puck %d\n", puckID);
    wakePuck(CANdev, puckID);
    
    /* Go live with zero torque */
   printf("Setting puck to MODE_IDLE\n");
    setProperty(CANdev, puckID, T, FALSE, 0);
    setProperty(CANdev, puckID, MODE, FALSE, MODE_IDLE);
    
    
}

/* Program entry point */
int main(int argc, char **argv)
{
   int   	err;        // Generic error variable for function calls

   
   /* Allow hard real time process scheduling for non-root users */
#ifdef RTAI   
   rt_allow_nonroot_hrt();
#else
   mlockall(MCL_CURRENT | MCL_FUTURE);
   /* Xenomai non-root scheduling is coming soon! */
#endif

   /* Initialize syslog */
   openlog("PUCK", LOG_CONS | LOG_NDELAY, LOG_USER);
   atexit((void*)closelog);

   /* Register the ctrl-c interrupt handler */
   signal(SIGINT, sigint_handler);

   /* Spin off the RT task to set up the CAN Bus.
    * RTAI priorities go from 99 (least important) to 1 (most important)
    * Xenomai priorities go from 1 (least important) to 99 (most important)
    */
   startDone = FALSE;
   btrt_thread_create(&rt_thd, "rtt", 45, (void*)rt_thread, NULL);
   while(!startDone)
      usleep(10000);

   progMode = TEST_TR;
   if(argc > 1){
      if(!strcmp(argv[1], "-c")){
         progMode = COLLECT_TR;
     }else if(!strcmp(argv[1], "-d")){
     	 progMode = TR_DATA;
     	 arg = argv;
     }
     
     
     	
   }
   
   /* Loop until Ctrl-C is pressed */
   
   
   btrt_thread_create(&main_thd, "main", 45, (void*)main_thread, NULL);
   
   while(1){
      usleep(10000);
   }
   
   /* We will never get here, but the compiler (probably) does not know this,
    * and it is expecting a return value from main(). Make it happy.
    */
   return(0); 
}

/**NOTE:
 * sprint_vn() and vect_n are from our own math library. 
 * See src/btsystem/btmath.c. They are specialized tools for handling vectors. 
 * The WAM's joint positions are stored in the "wam" data structure as a vector 
 * (wam->Jpos). sprint_vn() is like sprintf() for our vector data structure.
 */
