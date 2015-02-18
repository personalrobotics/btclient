p3util - Puck 3 utility program

Build:
$ make

Enumerate:
$ ./p3util -e

Set default properties:
$ ./p3util -p <id> <poles>
Note: If you do not enter <poles> then it defaults to 8-pole
Note: Proficio = 16 poles, WAM = 12 poles, Disney = 8 poles
Note: Sets GRPA, GRPB, GRPC, MT, TSTOP, HOLD, KP, KD, KI, IPNM, POLES, IKP, IKI, IKCOR

Set Serial Number:
$ ./p3util -s <id> <sn>

Get properties:
$ ./p3util -g <id>
Note: The property list is the same as the "default properties" above

Set ID & ROLE:
$ ./p3util -i <oldID> -l <newID> [-r <newROLE>]
Note: setting the ROLE is optional

Find offsets (IOFST, MOFST):
$ ./p3util -f <id>
Note: The Phase A stall torque is defined by the IKCOR value
Note: The MOFST value reported is the mechanical angle (MECH) % ctsPerElecCyc, so it should be consistent regardless of the rotor starting position

Tune the motor (using Korg nanoKontrol2):
$ ./p3util -t
Note: You must have a nanoKontrol2 device attached to your PC
