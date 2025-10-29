% Pick and place

%% 00 Connect to ROS (use your own masterhost IP address)
clc
clear
rosshutdown;
pause(2);       

%% Set IP address for master and node:
masterHostIP = "10.52.53.249";
nodeHostIP = "10.52.50.146"
rosinit(masterHostIP, 11311, "NodeHost",nodeHostIP);

%% ROS Class handle

% r will contains all publisher/subscriber/service/action/kinematic/tf info
disp("Creating Robot Handle...");
r = rosClassHandle_UR5e;

%% Options 

% Create global dictionary. Passed to all functions. Includes robot handle
keys   = ["debug", "rHandle"];
values = {      0, r};

% Instantiate the dictionary: values can be access via {}, i.e. optns{'key'}
disp("Creating dictionary...");
optns = dictionary(keys,values);  

%% 02 Reset the simulation

disp('Resetting the world...');
resetWorld(optns);      % reset models through a gazebo service

%% 03 Get Model Poses

type = 'gazebo'; % gazebo, ptcloud, cam, manual
disp('Getting object goal pose(s)...')

% Get models from Gazebo
models = getModels(optns);

% Number of models to pick (you can hard code or randomize)
n = 1; % n = randi([3 25]);, % n=25

% Manual Specification of fixed objects (may change from year-to-year)
rCan1 = [0.4, -0.5, 0.14, -pi/2, -pi 0];
rCan2 = [0.018, 0.66, 0.25, -pi/2, -pi 0];
rCan3 = [0.8, -0.03, 0.15, -pi/2, -pi, 0];
model_pos = [rCan1;rCan2;rCan3];

% Bin locations
greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];

%% 04 Get the poses
% Create a loop to do:
%   1. Get model pose in homogenous transformation format,
%   5. Go home

% Set the rate at which this loop should run
rate = rosrate(10);

% For Gazebo retrieved poses
if strcmp(type,'gazebo')

    % For n models to pick up
    for i=1:n

        %% 05.1 Get Model Pose
        
        % 05.1.1 Get Model Name
        model_name = models.ModelNames{23+randi([7,8])};

        % 05.1.2 Get Model pose
        fprintf('Picking up model: %s \n',model_name);

        % Get the pose and print only the position. 
        pose = get_model_pose(model_name,optns);

        % Print the ROS position for the model
        fprintf("The pose position (xyz) is: [ %0.2f, %0.2f, %0.2f ]\n", ...
            pose.Pose.Position.X,...
            pose.Pose.Position.Y,...
            pose.Pose.Position.Z);

        % Print the ROS quaternion for the model xyzw
        fprintf("The pose quaternion (xyzw) is: [ %0.2f, %0.2f, %0.2f, %0.2f ]\n", ...
    end
end
