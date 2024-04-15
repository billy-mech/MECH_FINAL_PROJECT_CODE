% this function is called by individualArmRound2.m 
% it will send the robot into the 'q_ready' position where the robot will
% have its elbow bent forward and toward the identified object

function qTravel3(mat_R_T_M)
    joint_state_sub = rossubscriber("/joint_states");
    ros_cur_jnt_state_msg = receive(joint_state_sub,1);
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                               'control_msgs/FollowJointTrajectory', ...
                                               'DataFormat', 'struct');
    traj_goal = rosmessage(pick_traj_act_client);
    qr = [atan2(-mat_R_T_M(1, 4), mat_R_T_M(2, 4)) 0 pi/2.3 -pi/2.3 0 0];
    traj_goal = convert2ROSPointVec(qr,ros_cur_jnt_state_msg.Name,1,1,traj_goal);

    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending Robot to "q_travel"...')
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        disp('First try failed... Trying again... Sending Robot to "q_travel"');
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end
    
    % Extract result
    res = res.ErrorCode;