% this function is called by main_level1.m
% it is in charge of performing one round (pick up and drop) for the arm

function individualArmRound2(object_number)

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

    % send the robot into the q_ready position (approved)
    disp('Sending Robot to the "q_ready" Position...')
    pause(1)
    qReady3(mat_R_T_M)
    disp('"q_ready" Goal Complete.')
    pause(5)

    % adjusting gripper rotation for horizontal objects
    if ismember(object_number, [28,30,22])  % horiztonal cans
        mat_R_T_M = mat_R_T_M*trotz(pi/4);
    elseif ismember(object_number, [31,39,35,36])  % horiztonal bottle
        mat_R_T_M = mat_R_T_M*trotz(pi/2);
    end

    % travel down to hover above object (approved)
    fprintf('Sending Robot to Hover Above %s...\n', model_name)
    pause(1)
    if ismember(object_number, [32,16,17])
        over_R_T_M = lift3(mat_R_T_M, 0.02);  % 0.05 is the vertical offset for rBottle2
    else
        over_R_T_M = lift3(mat_R_T_M, 0.1);  % 0.1 is the vertical offset all cans
    end
    traj_result = moveTo3(over_R_T_M,ops)
    fprintf('Hover Above %s Complete. \n', model_name)
    pause(5)

    % travel down to directly above object
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
    traj_result = moveTo3(mat_R_T_M,ops)
    fprintf('Descend to %s Complete. \n', model_name)
    pause(5)

    % close gripper onto object (approved) 
    if ismember(object_number, [20,23,24,25,26,27,29])  % upright can
        gripperManualUse3(0.23)
    elseif ismember(object_number, [21,28,30])  % horiztonal can
        gripperManualUse3(0.23125)
    elseif ismember(object_number, [31,32,39,35,36])  % horiztonal bottle
        gripperManualUse3(0.205)
    elseif ismember(object_number, [33,34,36,37,38])  % vertical bottles
        gripperManualUse3(0.514)
    elseif ismember(object_number, [12,13,14,15,16,17,18,19])  % pouches
        gripperManualUse3(0.54)
    end
    pause(1)

    % travel up to hover above where the object was (approved)
    fprintf('Sending Robot Up to Hover %s...\n', model_name)
    pause(1)
    if object_number == 32
        over_R_T_M = lift3(mat_R_T_M, 0.02);  % 0.05 is the vertical offset for rBottle2
    else
        over_R_T_M = lift3(mat_R_T_M, 0.1);  % 0.1 is the vertical offset all objects
    end
    traj_result = moveTo3(over_R_T_M,ops)
    fprintf('Hovering %s Complete. \n', model_name)
    pause(5)
    
    % send the robot into the q_travel position (approved)
    disp('Sending Robot to the "q_travel" Position...')
    pause(1)
    qTravel3(mat_R_T_M)
    disp('"q_travel" Goal Complete.')
    pause(5)

    % send robot to bin
    disp('Sending Robot to the Respective Bin...')
    pause(1)
    qBin3(object_number)
    disp('"q_Bin" Goal Complete.')
    pause(6)

    % opening grip to drop the object into respective bin
    gripperManualUse3(0)
    pause(1)

    % end print
    fprintf("%s Completed!\n", model_name)
    pause(5)

end