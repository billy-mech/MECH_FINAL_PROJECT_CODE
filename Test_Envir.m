% 00 Connect to ROS (use your own masterhost IP address)
    clc
    clear
    rosshutdown;
    masterhostIP = "192.168.2.128";
    rosinit(masterhostIP)

% 01 Go Home
    disp('Going home...');
    goHome('qr ');    % moves robot arm to a qr or qz start config
    disp('Resetting the world...');
    resetWorld;      % reset models through a gazebo service

%Robot = get_robot_object_pose_wrt_base_link("robot",1)
robot = loadrobot("universalUR5e","DataFormat","row");
canBox = collisionBox(0.07, 0.07, 0.12);
show(robot);
hold on

for i = [20:30]
    if ismember(i, [20:30])
        models = getModels;                                  % Extract gazebo model list
        model_name = models.ModelNames{i};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  
    
        fprintf('Creating Collision Boxes model: %s \n',model_name);
        position = get_model_pose(model_name);
        %mat_R_T_M = ros2matlabPose(position, 0.615, 1);
        [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name,1)
        env{i} = canBox
        env{i}.Pose(1:3, end) = mat_R_T_M(1:3,4);
        %env{i}.Pose(1:3, end) = [mat_R_T_M.Pose.Position.X, mat_R_T_M.Pose.Position.Y, mat_R_T_M.Pose.Position.Z]
        show(env{i})
        hold on
    elseif ismember(i, [22,23])
        disp('something went wrong')
    end
end


%env{1}.Pose(3, end) = -0.05;
%env{2}.Pose(1:3, end) = [0.1 0.2 0.8];
