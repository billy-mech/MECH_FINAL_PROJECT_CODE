% this function is called by individualArmRound2.m
% sends the robot to its home position -> q0 or 'q_ready' 
% (all joint angles are zero, and gripper is fully open)

function goHome3()
    joint_state_sub = rossubscriber("/joint_states");
    ros_cur_jnt_state_msg = receive(joint_state_sub,1);
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                               'control_msgs/FollowJointTrajectory', ...
                                               'DataFormat', 'struct');
    traj_goal = rosmessage(pick_traj_act_client);
    qr = [0 0 0 0 0 0];
    traj_goal = convert2ROSPointVec(qr,ros_cur_jnt_state_msg.Name,1,1,traj_goal);

    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending Robot to "q_home"...')
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        disp('First try failed... Trying again... Sending Robot to "q_home"');
        pause(1)
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end

    pause(3)
    gripperManualUse3(0)
    pause(1)

end

