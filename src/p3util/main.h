/* ======================================================================== *
 *  Module ............. btutil
 *  File ............... main.h
 *  Creation Date ...... 15 Feb 2003
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

char *statusTxt[] =
{
   "STATUS_OFFLINE",
   "STATUS_RESET",
   "STATUS_ERR",
   "STATUS_READY",
};

struct defaultStruct
{
   int *key;
   long val;
};

struct defaultStruct taterDefs[] =
{
   &TIE, 0,
   &ACCEL, 100,
   &AP, 0,
   &CT, 4096,
   &OT, 0,
   &CTS, 4096,
   &DP, 0,
   &MV, 100,
   &MCV, 100,
   &MOV, 100,
   &OT, 0,
   &HOLD, 0,
   &TSTOP, 0,
   &OTEMP, 60,
   &PTEMP, 0,
   &_DS, -256,
   &KP, 2000,
   &KD, 8000,
   &KI, 0,

   NULL, 0
};
long wamDefaultMT[] = {4860, 4860, 4860, 4320, 3900, 3900, 1600};

struct defaultStruct bh8Defs[] =
{
   &TIE, 0,
   &ACCEL, 200,
   &AP, 0,
   &CT, 195e3,
   &OT, 0,
   &CTS, 4096,
   &DP, 45e3,
   &MT, 2200,
   &MV, 200,
   &MCV, 200,
   &MOV, 200,
   &OT, 0,
   &HOLD, 0,
   &TSTOP, 50,
   &OTEMP, 60,
   &PTEMP, 0,
   &POLES, 6,
   &IKI, 204,
   &IKP, 500,
   &IKCOR, 102,
   &IOFF, 0,
   &IVEL, -75,
   &_DS, 25600,
   &KP, 500,
   &KD, 2500,
   &KI, 0,
   &IPNM, 20000,
   &HSG, 0,
   &LSG, 0,
   &GRPA, 0,
   &GRPB, 7,
   &GRPC, 5,

   NULL, 0
};

struct defaultStruct wraptorDefs[] =
{
   &TIE, 0,
   &ACCEL, 100,
   &AP, 0,
   &CT, 750,
   &CTS, 4096,
   &DP, 0,
   &MT, 7000,
   &MV, 2000,
   &MCV, 2000,
   &MOV, 2000,
   &DP, 0,
   &OT, 0,
   &CT, 195e3,
   &_DS, 2560,
   &KP, 2000,
   &KD, 8000,
   &KI, 0,
   &IKP, 4096,
   &IKI, 819,
   &IKCOR, 819,
   &GRPA, 0,
   &GRPB, 1,
   &GRPC, 4,
   &POLES, 6,
   &IPNM, 20000,

   NULL, 0
};

struct defaultStruct safetyDefs[] =
{
   &TIE, 0,
   &VOLTL1, 36,
   &VOLTL2, 30,
   &VOLTH1, 54,
   &VOLTH2, 57,
   &GRPA, 1,
   &GRPB, 2,
   &GRPC, 3,

   NULL, 0
};

struct fcnStruct{
   void (*f)(void *data);
   char title[64];
};

struct watchStruct{
   int puckID;
   int prop;
};
