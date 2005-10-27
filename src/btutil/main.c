/*======================================================================*
 *  Module .............btutil
 *  File ...............main.c
 *  Author .............Brian Zenowich
 *  Creation Date ......15 Feb 2003
 *  Addtl Authors ......
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/

/** \file btutil.c
 
Puck utilities:
 
Bus enumeration - Prints out what is alive
 
Bus enumeration and puck status - 
  Prints out all interesting puck values.
  
Puck Find motor offsets
 
Puck - Load WAM enumeration information.
 
Puck - Load new firmware
 
*/

/** Usage:
 
btutil [-c configfile] command [detail]
 
where command is:
 
  enum - List what is on the can bus and what their state is.
  stat - List paramers of interest
    all - dump all parameters
    init - dump parameters that are important to initial startup
    
  moffst # - Find motor offset for puck id #
  writefirmware # filename - write the specified firmware file
  writewaminfo # - write default wam enumeration info to puck id #
  copyparameters filename - read all parameters and store them to a file
  writeparameters filename - write all parameters stored in a file  
  
 
 
*/


#define toupper(c)      ( ((c >= 'a') && (c <= 'z')) ? c - ('a' - 'A') : c )
#define HOLE            (0x0000)
#if 0
#define SMOOTH_CT       (5)


#define REVOLUTIONS     (2)
#define STEP            (1)
#define TR_SAMPLE_TIME  (0.005)
#define WAIT_TIME       (0.0025)
#endif

#define TR_GAIN         (0.8)
#define GAIN_WINDOW     (1)
#define pi              (3.14159)

#define TEST_TR         (0)
#define COLLECT_TR      (1)

#include <stdio.h>
#include <syslog.h>
#include <inttypes.h>
#include <malloc.h>
#include <math.h>
#include <semaphore.h>

#include "btcan.h"
#include "btserial.h"

//extern  local_info_t    priv_data;
int     REVOLUTIONS_F, REVOLUTIONS_R, SMOOTH_CT;
int     STEP;
double  TR_SAMPLE_TIME;
double  WAIT_TIME;
double  kp, kd, ki;
int     compTorque;

PORT p;

typedef struct
{
    char s[20];
}
string20;



const    string20 statusTxt[]=
    {
        {"STATUS_OFFLINE"},
        {"STATUS_RESET"},
        {"STATUS_ERR"},
        {"STATUS_READY"},
    };

void PuckControlThread(void *data);
int firmwareDL(void);
void handleMenu(char c);
void showMenu(void);
void tensionCable(void);

void showMenu(void)
{
    printf("\nMENU");
    printf("\n--------");
    printf("\nE)numerate bus status");
    printf("\nF)ind Motor offset");
    printf("\nP)arameter defaults");
    printf("\nD)ownload firmware");
    //printf("\nC)hange puck ID");
    printf("\nT)ension cable");
    //printf("\nB)arrettHand firmware download");

    printf("\n\nYour Choice: ");
}

void handleMenu(char c)
{
    char        fname[32];
    int         node;
    int         err;
    uint64_t    time1, time2;
    float       delta;
    long        reply;
    long         status[MAX_NODES];
    int         i;
    double      cps;
    int         newID,dummy;
    long dat;


    switch(c) {
    case 'E':
        printf("\n\nCAN bus enumeration (status)\n");
        getBusStatus(0, status);
        for(i = 0; i < MAX_NODES; i++) {
            if(i % 2)
                printf("\t\tNode %2d: %s", i, statusTxt[status[i]+1].s);
            else
                printf("\nNode %2d: %s", i, statusTxt[status[i]+1].s);
        }
        printf("\n");
        break;
    case 'F':
        printf("\n\nSet puck MOFST\n");
        printf("\nPuckID: ");
        scanf("%d", &newID);
        wakePuck(0,newID);
        setProperty(0,newID,MODE,0,MODE_TORQUE);

        getProperty(0,newID,MOFST,&dat);
        printf("\n The old MOFST was:%d\n",dat);

        setProperty(0,newID,ADDR,0,32971);
        setProperty(0,newID,VALUE,0,1);
        printf("\nPress enter when the index pulse is found: ");
        scanf("%d", &dummy);
        setProperty(0,newID,ADDR,0,32970);
        getProperty(0,newID,VALUE,&dat);
        printf("\n The MOFST new is:%d\n",dat);
        setProperty(0,newID,MOFST,0,dat);
        setProperty(0,newID,SAVE,0,MOFST);
        printf("\nDone: ");
        printf("\n");
        break;
    case 'P':
        printf("\n\nSet defaults for puck ID: ");
        scanf("%d", &newID);
        wakePuck(0,newID);
        setProperty(0,newID,ACCEL,0,32);
        setProperty(0,newID,AP,0,0);
        setProperty(0,newID,CT,0,750);
        setProperty(0,newID,CTS,0,40960);
        setProperty(0,newID,DP,0,0);
        setProperty(0,newID,EN,0,0x00EE);
        setProperty(0,newID,GAIN1,0,0x1000);
        setProperty(0,newID,GAIN2,0,0x1000);
        setProperty(0,newID,GAIN3,0,0x1000);
        if(newID <= 4) { //4DOF
            setProperty(0,newID,IKCOR,0,1638);
            setProperty(0,newID,IKP,0,8192);
            setProperty(0,newID,IKI,0,3276);
            setProperty(0,newID,IPNM,0,2700);
        } else if(newID <= 7) { //Wrist
            setProperty(0,newID,IKCOR,0,819);
            setProperty(0,newID,IKP,0,4096);
            setProperty(0,newID,IKI,0,819);
            if(newID != 7)
                setProperty(0,newID,IPNM,0,4100);
            else
                setProperty(0,newID,IPNM,0,21400);

        }
        setProperty(0,newID,MT,0,990);
        setProperty(0,newID,OFFSET1,0,0);
        setProperty(0,newID,OFFSET2,0,0);
        setProperty(0,newID,OFFSET3,0,0);
        setProperty(0,newID,JIDX,0,newID);
        setProperty(0,newID,PIDX,0,((newID-1)%4)+1);
        setProperty(0,newID,PTEMP,0,0);
        setProperty(0,newID,SAVE,0,-1); // Save All
        break;
    case 'D':
        firmwareDL();
        break;
    case 'T':
	    tensionCable();
	break;
    case 'B':
	    BHandDL();
	break;
    default:

        break;
    }
}

int main( int argc, char **argv )
{
    char            c;
    int             err;

    /* Initialize syslogd */
    openlog("LOG_ERR",LOG_CONS | LOG_NDELAY, LOG_USER);
    syslog(LOG_ERR, "syslog initalized");

    /* Initialize CAN */
    if(err = initCAN(0)) {
        syslog(LOG_ERR, "initCAN returned err=%d", err);
    }

    /* Show Menu */
    showMenu();

    /* Get Choice */
    c = getchar();
    //printf("\nYou pressed ASCII: %d", c);
    c = toupper(c);

    /* Handle Menu */
    handleMenu(c);

    return(0);
}

int firmwareDL(void)
{
    char fn[32];
    FILE *fp;
    char which[8];
    int id;
    int len;
    unsigned char data[8];
    unsigned char sendData[8];
    int msgID;
    int i;
    int cnt;
    long status[MAX_NODES];
    char line[100];
    int rcpt;

    printf("Please enter *.tek file name: ");
    scanf("%s", fn);
    printf("Which puck ID [all");
    getBusStatus(0, status);
    cnt = 0;
    for(i = 1; i < MAX_NODES; i++) {
        if(status[i] >= 0 && i != SAFETY_MODULE) {
            printf(", %d", i);
            ++cnt;
        }
    }
    printf("]: ");
    scanf("%s", which);
    if(!(strcmp(which, "all"))) {
	    // FlashErase takes a varying amount of time
	    // Puck's download timeout is only 0.1s
	    printf("\n'All' download not yet supported\n");
	    exit(0);
        //msgID = GROUPID(0); 
    } else {
        msgID = atoi(which);
        cnt = 1;
    }

    // Open the file
    if((fp=fopen(fn, "r"))==NULL) {
        return(1);
    }
    setProperty(0, msgID, STAT, 0, 0); // Reset
    usleep(1000000); // Wait a sec
    setProperty(0, msgID, VERS, 0, 0x000000AA);
    // For each line in the file
    //sendData[0] = 0x80 | VERS;
    //sendData[1] = 0x00;
    while(fgets(line, 99, fp) != NULL) {
        i = 1;
        printf("%s", line);
        while(line[i] >= '0') {
            // Wait for n "Get VERS"
            for(rcpt = 0; rcpt < cnt; rcpt++) {
                while(canReadMsg(0, &id, &len, data, 1))
                    usleep(100);
            }
            // Send the byte
            sendData[0] = line[i];
            canSendMsg(0, msgID, 1, sendData, 1);
            ++i;
        }
    }
    fclose(fp);
    return(0);
}

void tensionCable(void){
	int motor;

	printf("\nTension Cable\nTension which motor: ");
	scanf("%d", &motor);
	setProperty(0,GROUPID(0),TORQ,FALSE,0);
       	setProperty(0,GROUPID(0),STAT,FALSE,STATUS_READY);
	usleep(250000);
	setProperty(0,GROUPID(0),MODE,FALSE,MODE_TORQUE);
	printf("\nPlease move cable to shaft end, then press <Enter>");
	scanf("%*s");
	setProperty(0,motor,TENSION,FALSE,1);
	setProperty(0,motor,TORQ,FALSE,500);
	printf("\nPlease rotate shaft until tensioner engages, "
			"then press <Enter>");
	scanf("%*s");
	setProperty(0,motor,TENSION,FALSE,0);
	setProperty(0,motor,TORQ,FALSE,2500);
	usleep(1000000);
	setProperty(0,motor,TORQ,FALSE,0);
	printf("\nPlease work the tension through the cable, "
			"then press <Enter>");
	scanf("%*s");
	setProperty(0,GROUPID(0),MODE,FALSE,MODE_IDLE);

}

/* Firmware download sample code.

   Note: ReadSerial(char *inputBuffer, int charCt); 
         WriteSerial(char *outputBuffer, int charCt); 
    ...must be created under your operating system.
*/
int ReadSerial(char *buf, int bytesToRead){
    int bytesRead;
    int totalRead = 0;
    long msec = 0;
    
    /** Read data from the serial port */
    do{
        serialRead(&p, buf, bytesToRead, &bytesRead);
        buf += bytesRead;
        totalRead += bytesRead;
        if(bytesToRead == totalRead)
            break;
        usleep(20000);
        msec += 20;
        if(msec == 1000){
		printf("ReadSerial timeout!\n");
            return(1);
	}
    }while(1);
    
    return(0);
}

WriteSerial(char *buf, int bytesToWrite){
    /** Write data to the serial port */
    serialWrite(&p, buf, bytesToWrite);
    return(bytesToWrite);
}

// Read characters, check for errors
char Read(void)
{
    char buf[10];

    //read character
    if( ReadSerial( buf, 1 ) )
        return 0;

    return buf[0];
}


// Write characters, check for errors
int Write( char ch )
{
    char buf[2] = " ";
    buf[0] = ch;

    //write character
    return( WriteSerial( buf, 1 ) );
        
    //return;
}


// Echo write
int EchoWrite(char ch)
{
    char test;
    int err;

    err = Write( ch );
   printf("i");fflush(stdout); 
    do{
        test = Read();
    }while( ch != test );
   printf("o");fflush(stdout);
    return(err);
}

int BHFirmwareDL(char *fname){
    char stype[3],shex[80], line[100];
    int num,opcode,temp_lo,temp_high,first;
    FILE *fhook;
    int total_bytes;
    long i;
    int count = 0;
    int lobyte, hibyte;
    static int ramarray[300];
    
    count=0;
    total_bytes=0;
    
    //Open the user-specified *.S19 file
    if((fhook=fopen(fname,"r")) == NULL){
        printf("\nFILE NOT FOUND.\n");
        return(1);
    }
    
    //Scan the length of the *.S19 file
    while (!feof(fhook)){
        fgets(line,85,fhook); 
        
        sscanf(line,"%2s%2x%2x%2x%76s",stype,&num,&temp_high,&temp_lo,shex); 
        if (strstr(stype,"S1")) {                        
            num-=3;     // subtract addr. and checksum from # of pairs 
            total_bytes+=num;
        }
    }
    fclose (fhook);
    
    Write('X'); //Write to MC68HC811 e<X>ternal EEPROM

    printf( "\nPower up the hand to begin download...\n" );
    int errcnt=0;
    while ( Read() != ':' && errcnt<50) errcnt++; //Wait for RESET

    if( errcnt>=50 ){
        printf( "\nDownload Failed\n" );
        return(1);
    }

    Write(255);  // send first trigger byte 
    
    strcpy(stype,"");
    first=1;
    printf("\nProgress: 0%");
    fflush(stdout);
    //Download the *.S19 file
    if((fhook=fopen(fname,"r")) == NULL)
        return(1);
        
    while (!feof(fhook)){
        fgets(line,85,fhook); //Read a line of data from *.S19 file
        if( feof(fhook) ) break;

        // basic line format, scan into variables 
        sscanf(line,"%2s%2x%2x%2x%76s",stype,&num,&temp_high,&temp_lo,shex);
	printf("%s %x %x %x %s\n", stype, num, temp_high, temp_lo,
			shex);
        lobyte=temp_lo; hibyte=temp_high;
        if (strstr(stype,"S1")) {  //If this is a DATA S-Record 
            //Update progress                   
            float per;
            num-=3;     // subtract addr. and checksum from # of pairs 
            per = (100*(double)(count+num)/(double)total_bytes)-1;
            if ( per < 0.0) per = 0.0; 
            count += num;
            printf("\rProgress: %3.0lf%%", per);
            fflush(stdout);
            for (i=0;i<num; i+=1){  // read pairs into ramarray 
                sscanf(shex+i*2,"%2x",&opcode);
                ramarray[i]=opcode;
            }

            //Pass the data on to Monitor in the BarrettHand
            //Monitor will write the data to external RAM
            Write(255);
            EchoWrite (lobyte);
            EchoWrite (hibyte);
            EchoWrite (num);
            Write(255);
            for (i=0; i<num; i++) {
                EchoWrite (ramarray[i]);
            }
        }
    }
    fclose (fhook);
    printf("\rProgress: 100%\n");

    return(0);
}

int BHandDL(void){

    char fn[32];
    char portlocation[32];
    int err;
    
    // Ask for *.S19 file
    printf("\nBarrettHand Firmware Download\nPlease enter firmware (*.S19) filename: ");
    scanf("%s", fn);
    
    // Ask for port
    printf("\nPlease enter port (ex: /dev/ttyS0): ");
    scanf("%s", portlocation);
    
    /** Open serial port */
    err =  serialOpen(&p, portlocation);
    if(err){
	    printf("\nError opening port!\n");
	    exit(0);
    }

    /** Set the baud rate */
    serialSetBaud(&p, 9600);

    if(err = BHFirmwareDL(fn)){
        printf("\nDownload failed! Err = %d\n", err);
        exit(0);
    }
    
    printf("\n **** Download Complete! ****\n");
    
    /** Close serial port */
    serialClose(&p);
    
    return(0);
}
