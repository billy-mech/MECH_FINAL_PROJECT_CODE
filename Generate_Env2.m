function env = Generate_Env2

% % 01 Go Home
%     disp('Going home...');
%     goHome('qr ');    % moves robot arm to a qr or qz start config
%     disp('Resetting the world...');
%     resetWorld;      % reset models through a gazebo service

%Robot = get_robot_object_pose_wrt_base_link("robot",1)
robot = loadrobot("universalUR5e","DataFormat","row");
canBox = collisionBox(0.062, 0.062, 0.14);
bottleBox = collisionBox(0.062, 0.062, 0.2);
pouchBox = collisionBox(0.03, 0.03, 0.03);
scaleBox = collisionBox(0.3, 0.3, 0.1);
box2Box = collisionBox(0.12, 0.2, 0.16);
box1Box = collisionBox(0.16, 0.2, 0.14);
unitBox = collisionBox(0.4, 0.24, 0.1);
tableBox = collisionBox(0.8, 1.5, 0.53);
wcaseBox = collisionBox(0.5, 0.1, 0.4)

%% Set up robot in Matlab
qn = [pi/2 -pi/2 0 0 0 0];
robot.show(qn)

W_T_R = get_model_pose('robot')
%show(robot);
hold on

for i = [0:39]
    if ismember(i, [31:39])     %All Bottles
        models = getModels;                                  % Extract gazebo model list
        model_name = models.ModelNames{i};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  
    
        fprintf('Creating Collision Boxes model: %s \n',model_name);
        position = get_model_pose(model_name);
        [mat_R_T_G, mat_R_T_M] = get_object_pose_Zac(model_name);
        env{i} = bottleBox;
        env{i}.Pose = mat_R_T_M;
        show(env{i})
        hold on
    elseif ismember(i, [20:30]) %All Cans
        models = getModels;                                  % Extract gazebo model list
        model_name = models.ModelNames{i};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  
    
        fprintf('Creating Collision Boxes model: %s \n',model_name);
        position = get_model_pose(model_name);
        [mat_R_T_G, mat_R_T_M] = get_object_pose_Zac(model_name);
        %mat_R_T_M(3,4) =  (mat_R_T_M(3,4)-0.18);
        
        env{i}= canBox;
      
        %env{i}.Pose(1:3,1:3) = eul2tform([-pi/2 -pi/2 0]); % ZYX axis

        %canBox = collisionBox(0.062, 0.062, 0.14, "Pose",mat_R_T_M);
        %env{i} = canBox;
       
        %env{i}.Pose = mat_R_T_M;
        %env{i}.Pose(1:3, end) = [mat_R_T_M.Pose.Position.X, mat_R_T_M.Pose.Position.Y, mat_R_T_M.Pose.Position.Z]
        show(env{i})
        hold on
    elseif ismember(i, [12:19]) %All Pouches
        models = getModels;                                  % Extract gazebo model list
        model_name = models.ModelNames{i};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  
    
        fprintf('Creating Collision Boxes model: %s \n',model_name);
        position = get_model_pose(model_name);
        [mat_R_T_G, mat_R_T_M] = get_object_pose_Zac(model_name);
        env{i} = pouchBox;
        %mat_R_T_M(3,4) =  (mat_R_T_M(3,4)-0.18);
        env{i}.Pose = mat_R_T_M;
        %env{i}.Pose(1:3, end) = [mat_R_T_M.Pose.Position.X, mat_R_T_M.Pose.Position.Y, mat_R_T_M.Pose.Position.Z]
        show(env{i})
        hold on
    elseif ismember(i, [11])    %Scale
        models = getModels;                                  % Extract gazebo model list
        model_name = models.ModelNames{i};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  
    
        fprintf('Creating Collision Boxes model: %s \n',model_name);
        position = get_model_pose(model_name);
        [mat_R_T_G, mat_R_T_M] = get_object_pose_Zac(model_name);
        env{i} = scaleBox;
        %mat_R_T_M(3,4) =  (mat_R_T_M(3,4)-0.18);
        env{i}.Pose = mat_R_T_M;
        %env{i}.Pose(1:3, end) = [mat_R_T_M.Pose.Position.X, mat_R_T_M.Pose.Position.Y, mat_R_T_M.Pose.Position.Z]
        show(env{i})
        hold on
    elseif ismember(i, [10])    %Scale
        models = getModels;                                  % Extract gazebo model list
        model_name = models.ModelNames{i};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  
    
        fprintf('Creating Collision Boxes model: %s \n',model_name);
        position = get_model_pose(model_name);
        [mat_R_T_G, mat_R_T_M] = get_object_pose_Zac(model_name);
        env{i} = box2Box;
        % mat_R_T_M(3,4) =  (mat_R_T_M(3,4)-0.18);
        env{i}.Pose = mat_R_T_M;
        %env{i}.Pose(1:3, end) = [mat_R_T_M.Pose.Position.X, mat_R_T_M.Pose.Position.Y, mat_R_T_M.Pose.Position.Z]
        show(env{i})
        hold on
     elseif ismember(i, [8])    %Scale
        models = getModels;                                  % Extract gazebo model list
        model_name = models.ModelNames{i};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  
    
        fprintf('Creating Collision Boxes model: %s \n',model_name);
        position = get_model_pose(model_name);
        [mat_R_T_G, mat_R_T_M] = get_object_pose_Zac(model_name);
        mat_R_T_M(1,4) = mat_R_T_M(1,4)-0.2
        env{i} = wcaseBox;
        % mat_R_T_M(3,4) =  (mat_R_T_M(3,4)-0.18);
        env{i}.Pose = mat_R_T_M;
        %env{i}.Pose(1:3, end) = [mat_R_T_M.Pose.Position.X, mat_R_T_M.Pose.Position.Y, mat_R_T_M.Pose.Position.Z]
        show(env{i})
        hold on
    elseif ismember(i, [9])    %Scale
        models = getModels;                                  % Extract gazebo model list
        model_name = models.ModelNames{i};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  
    
        fprintf('Creating Collision Boxes model: %s \n',model_name);
        position = get_model_pose(model_name);
        [mat_R_T_G, mat_R_T_M] = get_object_pose_Zac(model_name);
        env{i} = box1Box;
        %mat_R_T_M(3,4) =  (mat_R_T_M(3,4)-0.18);
        env{i}.Pose = mat_R_T_M;
        %env{i}.Pose(1:3, end) = [mat_R_T_M.Pose.Position.X, mat_R_T_M.Pose.Position.Y, mat_R_T_M.Pose.Position.Z]
        show(env{i})
        hold on
    elseif ismember(i, [4])    %Scale
        models = getModels;                                  % Extract gazebo model list
        model_name = models.ModelNames{i};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  
    
        fprintf('Creating Collision Boxes model: %s \n',model_name);
        position = get_model_pose(model_name);
        [mat_R_T_G, mat_R_T_M] = get_object_pose_Zac(model_name);
        env{i} = unitBox;
        env{i}.Pose = mat_R_T_M;
        show(env{i})
        hold on
    elseif ismember(i, [2, 3])    %Scale
        models = getModels;                                  % Extract gazebo model list
        model_name = models.ModelNames{i};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  
        fprintf('Creating Collision Boxes model: %s \n',model_name);
        position = get_model_pose(model_name);
        [mat_R_T_G, mat_R_T_M] = get_object_pose_Zac(model_name);
        env{i} = tableBox;
        mat_R_T_M(3,4) =  (mat_R_T_M(3,4)+0.25);
        env{i}.Pose= mat_R_T_M;
        show(env{i})
        hold on
    else
        disp('something went wrong')
    end
end

disp('Enviroment Has Been Built')