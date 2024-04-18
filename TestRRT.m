% planner 

robot = loadrobot("universalUR5e","DataFormat","row");

env = {collisionBox(0.5, 0.5, 0.05) collisionBox(0.07, 0.07, .12)};
env{1}.Pose(3, end) = -0.05;
env{2}.Pose(1:3, end) = [0.1 0.8 0.0];

startConfig = [0 0 pi/2 -pi/2 0 0];
goalConfig = [0 0 0 0 0 0];

rrt = manipulatorRRT(robot,env);
rrt.SkippedSelfCollisions = "parent"

rng(0)
path = plan(rrt,startConfig,goalConfig);

interpPath = interpolate(rrt,path);
clf
for i = 1:20:size(interpPath,1)
    show(robot,interpPath(i,:));
    hold on
end
show(env{1})
show(env{2})
hold off

% show(robot, goalConfig);
% hold on
% show(env{1})
% show(env{2})