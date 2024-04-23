% =========================================================================
% This is the Mechatronics Final Project MAIN Code
% Inputs:
% None
% Outputs:
% None
% This is the MAIN matlab file that will be calling the
%       Test_Envir.m function (which makes the environment for the robot to avoid)
%       individualArmRound2.m function (which calls many functions itself)
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

% loading environment
disp('Loading Environment...')
env = Test_Envir();
% taking all objects from the env cell array, and placing them into the new
%   environment cell array, effectively removing all non-collision-box 
%   objects
env_new = {};
for i = 1:numel(env)
    if ~isa(env{i}, 'double')
        env_new{end+1} = env{i};
    end
end
disp('Environment Complete.')
pause(1)

% qBin to start (currently unnecessary)
% qBin3(rCan1_number)

% pouches 
individualArmRound2(pouch1, env_new)  % approved
pause(1)
individualArmRound2(pouch2, env_new)  % approved
pause(1)
individualArmRound2(pouch3, env_new)  % approved
pause(1)
individualArmRound2(pouch4, env_new)  % approved
pause(1)
individualArmRound2(pouch5, env_new)  % approved
pause(1)
individualArmRound2(pouch6, env_new)  % approved
pause(1)
individualArmRound2(pouch7, env_new)  % approved
pause(1)
individualArmRound2(pouch8, env_new)  % approved
pause(1)

% cans (green bin)
individualArmRound2(rCan1_number, env_new)  % approved
pause(1)
% individualArmRound2(rCan2_number, env_new)  % approved
% pause(1)
% individualArmRound2(rCan3_number, env_new)  % approved
% pause(1)
individualArmRound2(yCan1_number, env_new)  % approved
pause(1)
% individualArmRound2(yCan2_number, env_new)  % (horizontal) approved
% pause(1)
% individualArmRound2(yCan3_number, env_new)  % approved
% pause(1)
% individualArmRound2(yCan4_number, env_new) %  (horizontal) approved
% pause(1)
% individualArmRound2(gCan1_number, env_new)  % approved
% pause(1)
individualArmRound2(gCan2_number, env_new)  % (horizontal) approved
pause(1)
% individualArmRound2(gCan3_number, env_new)  
% pause(1)
% individualArmRound2(gCan4_number, env_new)  % approved
% pause(1)

% bottles (blue bin)
individualArmRound2(rBottle1_number, env_new)  % (horizontal) approved
pause(1)
% individualArmRound2(rBottle2_number, env_new)  % (horizontal) approved
% pause(1)
% individualArmRound2(bBottle1_number, env_new)
% pause(1)
% individualArmRound2(bBottle2_number, env_new)  % approved
% pause(1)
% individualArmRound2(bBottle3_number, env_new)  % (horizontal) approved, stuck
% pause(1)
% individualArmRound2(yBottle1_number, env_new)  % approved
% pause(1)
% individualArmRound2(yBottle2_number, env_new)  % approved
% pause(1)
individualArmRound2(yBottle3_number, env_new)  % approved
pause(1)
% individualArmRound2(yBottle4_number, env_new)  % (horizontal)
% pause(1)
