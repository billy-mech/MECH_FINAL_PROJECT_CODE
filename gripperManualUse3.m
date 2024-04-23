% =========================================================================
% it controls the gripper to open or close by the specified argument
%
% Inputs:
% 	gripper_distance_value - Number between 0 and 0.8 that sets the gripper value
%
% Output
% 	None
% =========================================================================

function gripperManualUse3(gripper_distance_value)
    grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                                  'control_msgs/FollowJointTrajectory',...
                                                  'DataFormat','struct');
    grip_msg = rosmessage(grip_action_client);
    gripPos = gripper_distance_value; 
    grip_goal = packGripGoal_struct4(gripPos,grip_msg);
    disp('Sending grip goal...');
    pause(1)
    sendGoalAndWait(grip_action_client,grip_goal);
    pause(2)
    disp('Grip goal complete.');
    pause(2)
end

