% =========================================================================
% This is the Mechatronics Final Project MAIN Code
% This is the MAIN matlab file that will be calling the
%       individual_arm_round.m function (which calls many functions itself)
% Make sure to initialize ROS before beginning the program
% RRT :)
% =========================================================================

clc;

% defining object numbers
pouch1 = 12;
pouch2 = 13;
pouch3 = 14;
pouch4 = 15;
pouch5 = 16;
pouch6 = 17;
pouch7 = 18;
pouch8 = 19;
gCan1_number = 20;
gCan2_number = 21;  % horizontal
gCan3_number = 22;  % get this can after the red can on top of it
gCan4_number = 23;
rCan1_number = 24;
rCan2_number = 25;
rCan3_number = 26;
yCan1_number = 27;
yCan2_number = 28;  % horizontal
yCan3_number = 29;
yCan4_number = 30;  % horizontal
rBottle1_number = 31;  % horizontal
rBottle2_number = 32;  % horizontal
bBottle1_number = 33;
bBottle2_number = 34;
bBottle3_number = 35;  % horizontal
yBottle1_number = 36;
yBottle2_number = 37;
yBottle3_number = 38;
yBottle4_number = 39;  % horizontal

% opening print
disp('Hello User! Starting Robot...')
pause(1)

% cans (green bin)
individualArmRound2(rCan1_number)
pause(1)
individualArmRound2(rCan2_number)
pause(1)
individualArmRound2(rCan3_number)
pause(1)
individualArmRound2(yCan1_number) 
pause(1)
individualArmRound2(yCan2_number)  % (horizontal)
pause(1)
individualArmRound2(yCan3_number) 
pause(1)
individualArmRound2(yCan4_number) %  (horizontal)
pause(1)
individualArmRound2(gCan1_number)  % (horizontal)
pause(1)
individualArmRound2(gCan2_number)  
pause(1)
individualArmRound2(gCan3_number)
pause(1)
individualArmRound2(gCan4_number)
pause(1)

% bottles (blue bin)
individualArmRound2(rBottle1_number)  % (horizontal)
pause(1)
individualArmRound2(rBottle2_number)  % (horizontal)
pause(1)
individualArmRound2(bBottle1_number)
pause(1)
individualArmRound2(bBottle2_number) 
pause(1)
individualArmRound2(bBottle3_number)  % (horizontal)
pause(1)
individualArmRound2(yBottle1_number)
pause(1)
individualArmRound2(yBottle2_number)
pause(1)
individualArmRound2(yBottle3_number) 
pause(1)
individualArmRound2(yBottle4_number)  % (horizontal)
pause(1)

% pouches 
individualArmRound2(pouch1)  
pause(1)
individualArmRound2(pouch2)  
pause(1)
individualArmRound2(pouch3)
pause(1)
individualArmRound2(pouch4) 
pause(1)
individualArmRound2(pouch5)  
pause(1)
individualArmRound2(pouch6)
pause(1)
individualArmRound2(pouch7)
pause(1)
individualArmRound2(pouch8) 
pause(1)
