% this function is called by individualArmRound2.m 
% it will send the robot into the 'q_bin' position where the robot will
% have its elbow bent forward and toward the proper bin
% cans and pouches go to the green bin, bottles go to blue bin

function qBin3(object_number_i_f)
    joint_state_sub = rossubscriber("/joint_states");
    ros_cur_jnt_state_msg = receive(joint_state_sub,1);
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                               'control_msgs/FollowJointTrajectory', ...
                                               'DataFormat', 'struct');
    traj_goal = rosmessage(pick_traj_act_client);
    if object_number_i_f >= 12 && object_number_i_f <= 30  % pouches and cans go to green bin
        qBin = [5/8*pi 0 pi/2.3 -pi/2.3 0 0];
    else  % bottles go to blue bin
        qBin = [-0.8*pi 0 pi/2.3 -pi/2.3 0 0];
    end
    traj_goal = convert2ROSPointVec(qBin,ros_cur_jnt_state_msg.Name,1,1,traj_goal);

    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending Robot to "q_ready"...')
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        disp('First try failed... Trying again... Sending Robot to "q_ready"');
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end
    
    % Extract result
    res = res.ErrorCode;