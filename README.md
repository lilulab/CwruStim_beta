# CwruStim_beta
CWRU HNPv2 Project Muscle Stimulation UART Interface Library Beta Version

**Git Clone this repo to your local work folder. Branch it if you need to modify it.**

* CwruStim_beta.ino - Beta Code for muscle stim board for HNPv2 Project.

* CwruStim.cpp - Library source code for muscle stim board for HNPv2 Project.

* CwruStim.h - Library head file for muscle stim board for HNPv2 Project.

**Note**

The UECU Message Handbook I made, and it would be the most helpful doc if anyone want to work on the stim board communication in the future:
https://goo.gl/s20iH4

The CwruStim_beta code is fully functional, we are able to setup 4 bipolar channels, and change the pulse width and amplitude on the fly. Beside the code is able to connect 1-3 stim boards using Hybrid ECB, by create multiple Stim Class object.

I call it beta version, because I still need to confirm with Jeremy about:

1. Prove my assumption about the autonumbering convention for schedule and events inside the stim board controller.
2. Verify the correct usage of port_chn_id. 
3. Wait for more docs and sample code from Tina to make sure all my guesses and decoding are right.

Although, this is a working and functioning version. Please use it with caution, before I got more tech details about what's going on inside the magic black box. Enjoy!


