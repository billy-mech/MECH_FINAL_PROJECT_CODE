% this function is called by individualArmRound2.m
% it controls the gripper closing, and stops when it senses a force

function gripperCloseForceSensing3()
    forceSensorSub = rossubscriber('/force_sensor_topic');
    forceThreshold = 4;

    gripperClosed = false;
    
    % Main loop for gripper control
    while true
        % Get latest force/torque data from the gripper sensor
        forceMsg = receive(forceSensorSub, 1);  % Receive sensor data
        
        % Extract force values from the message
        forceX = forceMsg.Wrench.Force.X;  % X-axis force
        forceY = forceMsg.Wrench.Force.Y;  % Y-axis force
        forceZ = forceMsg.Wrench.Force.Z;  % Z-axis force
        
        % Calculate magnitude of the total force
        totalForce = norm([forceX, forceY, forceZ]);  % Magnitude of force vector
        
        % Check if the total force exceeds the threshold
        if totalForce > forceThreshold && ~gripperClosed
            % Command UR5e gripper to stop closing (send ROS command)
            % Example ROS command (replace with actual gripper control command):
            % rosmsg = rosmessage('your_gripper_control_msg');
            % rosmsg.Action = 'stop';
            % send(your_gripper_control_publisher, rosmsg);
            
            disp('Gripper stopped closing due to sensed force.');
            gripperClosed = true;  % Gripper is now closed/stopped
        end
        
        % Add optional logic to handle other conditions or actions
        
        % Break out of the loop (for demonstration purposes)
        % Replace this with actual termination condition based on your application
        if gripperClosed
            break;  % Exit loop once gripper is closed/stopped
        end
        
        % Pause briefly before next iteration (adjust as needed)
        pause(0.1);  % Pause to control loop frequency
    end
end

