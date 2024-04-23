% =========================================================================
% This is the Mechatronics Final Project Secondary Code
% Inputs:
% object_number (double) - number of the object to be sorted
% env_i_f {cell} - environment of collision boxes
% There are no Outputs
% This is the matlab file that will be calling:
%       lift3 (Increases the z-position of mat_current by z_offset)
%       moveTo3 (this function moves the arm to a specified transformation matrix)
% 	    gripperManualUse3 (it controls the gripper to open or close by the specified argument)
% =========================================================================

function individualArmRound2(object_number, env_i_f)

    % defining ops dictionary (approved)
    ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.1;   % Vertical offset for top-down approach
    ops("traj_duration")       = 2;     % Traj duration (secs)   

    % getting pose for object (approved)
    models = getModels;      % Extract gazebo model list
    model_name = models.ModelNames{object_number}; 
    fprintf('Calculating Pose for %s...\n', model_name)
    pause(1)
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name);

    % finding current joint configuration (approved)
    joing_state_sub_in_indRound2 = rossubscriber("/joint_states");
    ros_current_joint_state_msg_in_indRound2 = receive(joing_state_sub_in_indRound2,1);
    matlab_current_joint_state_msg_in_indRound2 = zeros(1,6);
    matlab_current_joint_state_msg_in_indRound2(1,1) = ros_current_joint_state_msg_in_indRound2.Position(4,1);
    matlab_current_joint_state_msg_in_indRound2(1,2) = ros_current_joint_state_msg_in_indRound2.Position(3,1);
    matlab_current_joint_state_msg_in_indRound2(1,3) = ros_current_joint_state_msg_in_indRound2.Position(1,1);
    matlab_current_joint_state_msg_in_indRound2(1,4) = ros_current_joint_state_msg_in_indRound2.Position(5,1);
    matlab_current_joint_state_msg_in_indRound2(1,5) = ros_current_joint_state_msg_in_indRound2.Position(6,1);
    matlab_current_joint_state_msg_in_indRound2(1,6) = ros_current_joint_state_msg_in_indRound2.Position(7,1)

    % adjusting gripper rotation for horizontal objects (approved)
    if ismember(object_number, [28,30,22])  % horiztonal cans
        mat_R_T_M = mat_R_T_M*trotz(pi/4);
    elseif ismember(object_number, [31,39,35,36])  % horiztonal bottle
        mat_R_T_M = mat_R_T_M*trotz(pi/2);
    end

    % finding hover position (approved)
    if ismember(object_number, [32,16,17])
        over_R_T_M = lift3(mat_R_T_M, 0.02);  % 0.05 is the vertical offset for rBottle2
    else
        over_R_T_M = lift3(mat_R_T_M, 0.1);  % 0.1 is the vertical offset all cans
    end

    % finding joint state for q_hover (approved)
    ur5e = loadrobot("universalUR5e",DataFormat="row");
    ik_joint_angles = convertPoseTraj2JointTraj4_new_RRT(ur5e,over_R_T_M,ops('toolFlag'));

    % RRT (approved)
    startConfig = matlab_current_joint_state_msg_in_indRound2;
    goalConfig = ik_joint_angles;
    rrt = manipulatorRRT(ur5e,env_i_f);
    rrt.SkippedSelfCollisions = "parent";
    rng(0)
    path = plan(rrt,startConfig,goalConfig)

    % sending robot to hover position (RRT midpoint) (approved)
    fprintf('Sending Robot to RRT Midpoint for %s...\n', model_name)
    pause(1)
    joint_state_sub = rossubscriber("/joint_states");
    ros_cur_jnt_state_msg = receive(joint_state_sub,1);
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                               'control_msgs/FollowJointTrajectory', ...
                                               'DataFormat', 'struct');
    traj_goal = rosmessage(pick_traj_act_client);
    % q_hover = ik_joint_angles;
    traj_goal = convert2ROSPointVec(path(2,:),ros_cur_jnt_state_msg.Name,1,1,traj_goal);
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending Robot to "q_RRT1"...')
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        disp('First try failed... Trying again... Sending Robot to "q_RRT1"');
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end
    fprintf('Robot to RRT Midpoint for %s Complete. \n', model_name)
    pause(5)

    % sending robot to hover position (RRT endpoint) (approved)
    fprintf('Sending Robot to RRT Endpoint for %s...\n', model_name)
    pause(1)
    joint_state_sub = rossubscriber("/joint_states");
    ros_cur_jnt_state_msg = receive(joint_state_sub,1);
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                               'control_msgs/FollowJointTrajectory', ...
                                               'DataFormat', 'struct');
    traj_goal = rosmessage(pick_traj_act_client);
    % q_hover = ik_joint_angles;
    traj_goal = convert2ROSPointVec(path(3,:),ros_cur_jnt_state_msg.Name,1,1,traj_goal);
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending Robot to "q_RRT2"...')
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        disp('First try failed... Trying again... Sending Robot to "q_RRT2"');
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end
    fprintf('Robot to RRT Endpoint for %s Complete. \n', model_name)
    pause(5)

    % travel down to directly above object (approved)
    fprintf('Sending Robot to Descend to %s...\n', model_name)
    pause(1)
    if ismember(object_number, [21,28,30,31,32])  % horizontal cans
        mat_R_T_M(3,4) = mat_R_T_M(3,4) - 0.05;
    elseif ismember(object_number, [31,32,39,35,36])  % horiztonal bottles
        mat_R_T_M(3,4) = mat_R_T_M(3,4) - 0.045;
    elseif ismember(object_number, [33,34,36,37,38])  % vertical bottles
        mat_R_T_M(3,4) = mat_R_T_M(3,4) + 0.06;
    elseif ismember(object_number, [12,13,14,15,16,17,18,19])  % pouches
        mat_R_T_M(3,4) = mat_R_T_M(3,4) - 0.03;
    end
    traj_result = moveTo3(mat_R_T_M,ops);
    fprintf('Descend to %s Complete. \n', model_name)
    pause(5)

    % close gripper onto object (approved) 
    if ismember(object_number, [20,23,24,25,26,27,29])  % upright can
        gripperManualUse3(0.23)
    elseif ismember(object_number, [21,28,30])  % horiztonal can
        gripperManualUse3(0.23125)
    elseif ismember(object_number, [31,39,36])  % horiztonal bottle
        gripperManualUse3(0.205)
    elseif ismember(object_number, [32,35])
        gripperManualUse3(0.21)
    elseif ismember(object_number, [33,34,38])  % vertical bottles
        gripperManualUse3(0.514)
    elseif ismember(object_number, 37)  % one vertical bottle yBottle2
        gripperManualUse3(0.525)
    elseif ismember(object_number, [12,13,14,15,16,17,18,19])  % pouches
        gripperManualUse3(0.54)
    end
    pause(1)

    % travel up to hover above where the object was (approved)
    fprintf('Sending Robot Up to Hover %s...\n', model_name)
    pause(1)
    if object_number == 32
        over_R_T_M = lift3(mat_R_T_M, 0.02);  % 0.02 is the vertical offset for rBottle2
    elseif ismember(object_number, [28,30,21])
        over_R_T_M = lift3(mat_R_T_M, 0.2);  % 0.2 is the vertical offset for yCan2 (horizontal)
    else
        over_R_T_M = lift3(mat_R_T_M, 0.1);  % 0.1 is the vertical offset all objects
    end
    traj_result = moveTo3(over_R_T_M,ops)
    fprintf('Hovering %s Complete. \n', model_name)
    pause(5)

    % finding current joint configuration (approved)
    joing_state_sub_in_indRound2 = rossubscriber("/joint_states");
    ros_current_joint_state_msg_in_indRound2 = receive(joing_state_sub_in_indRound2,1)
    matlab_current_joint_state_msg_in_indRound2 = zeros(1,6);
    matlab_current_joint_state_msg_in_indRound2(1,1) = ros_current_joint_state_msg_in_indRound2.Position(4,1);
    matlab_current_joint_state_msg_in_indRound2(1,2) = ros_current_joint_state_msg_in_indRound2.Position(3,1);
    matlab_current_joint_state_msg_in_indRound2(1,3) = ros_current_joint_state_msg_in_indRound2.Position(1,1);
    matlab_current_joint_state_msg_in_indRound2(1,4) = ros_current_joint_state_msg_in_indRound2.Position(5,1);
    matlab_current_joint_state_msg_in_indRound2(1,5) = ros_current_joint_state_msg_in_indRound2.Position(6,1);
    matlab_current_joint_state_msg_in_indRound2(1,6) = ros_current_joint_state_msg_in_indRound2.Position(7,1)
    
    % send the robot into the q_travel position (approved) (currently unnecessary)
    % disp('Sending Robot to the "q_travel" Position...')
    % pause(1)
    % qTravel3(mat_R_T_M)
    % disp('"q_travel" Goal Complete.')
    % pause(5)

    % RRT (approved)
    startConfig = matlab_current_joint_state_msg_in_indRound2;
    if object_number >= 12 && object_number <= 30  % pouches and cans go to green bin 
        goalConfig = [5/8*pi 0 pi/2.3 -pi/2.3 0 0];
    else  % bottles go to blue bin
        goalConfig = [-0.8*pi 0 pi/2.3 -pi/2.3 0 0];
    end
    rrt = manipulatorRRT(ur5e,env_i_f);
    rrt.SkippedSelfCollisions = "parent";
    rng(0)
    path = plan(rrt,startConfig,goalConfig)

    % sending robot to hover position (RRT midpoint) (approved)
    fprintf('Sending Robot to RRT Midpoint for %s...\n', model_name)
    pause(1)
    joint_state_sub = rossubscriber("/joint_states");
    ros_cur_jnt_state_msg = receive(joint_state_sub,1);
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                               'control_msgs/FollowJointTrajectory', ...
                                               'DataFormat', 'struct');
    traj_goal = rosmessage(pick_traj_act_client);
    % q_hover = ik_joint_angles;
    traj_goal = convert2ROSPointVec(path(2,:),ros_cur_jnt_state_msg.Name,1,1,traj_goal);
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending Robot to "q_RRT1"...')
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        disp('First try failed... Trying again... Sending Robot to "q_RRT1"');
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end
    fprintf('Robot to RRT Midpoint for %s Complete. \n', model_name)
    pause(5)

    % sending robot to hover position (RRT endpoint) (approved)
    fprintf('Sending Robot to RRT Endpoint for %s...\n', model_name)
    pause(1)
    joint_state_sub = rossubscriber("/joint_states");
    ros_cur_jnt_state_msg = receive(joint_state_sub,1);
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                               'control_msgs/FollowJointTrajectory', ...
                                               'DataFormat', 'struct');
    traj_goal = rosmessage(pick_traj_act_client);
    % q_hover = ik_joint_angles;
    traj_goal = convert2ROSPointVec(path(3,:),ros_cur_jnt_state_msg.Name,1,1,traj_goal);
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending Robot to "q_RRT2"...')
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        disp('First try failed... Trying again... Sending Robot to "q_RRT2"');
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end
    fprintf('Robot to RRT Endpoint for %s Complete. \n', model_name)
    pause(5)

    % send robot to bin (approved) (currently unnecessary)
    % disp('Sending Robot to the Respective Bin...')
    % pause(1)
    % qBin3(object_number)
    % disp('"q_Bin" Goal Complete.')
    % pause(6)

    % opening grip to drop the object into respective bin (approved)
    gripperManualUse3(0)
    pause(1)

    % end print (approved)
    fprintf("%s Completed!\n", model_name)
    pause(5)

end