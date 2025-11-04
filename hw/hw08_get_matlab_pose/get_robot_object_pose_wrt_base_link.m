function [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name,...
                                                                      get_robot_gripper_pose_flag,...
                                                                      optns)
%--------------------------------------------------------------------------
% Extract robot-end-effector and model (object) poses wrt to base_link. 
%
% 1a. Use gazebo service to extract both: (i) robot's base and (ii) model (object) poses wrt to Gazebo's world origin.
% 1b. Transform both coordinate frames from Gazebo's world origin to the base link.
% Get starting pose of robot and use its orientation to pick up object
%
% Input:
% model_name (string) - name of model available in gazebo simulation
% get_robot_gripper_pose_flag (double) - 1|0 to indicate if we want to compute robot gripper (uses tf tform)
%
% Output: 
% - mat_R_T_G [4x4] double - transformation from robot base_link to tip/End-Effector
% - mart_R_T_M [4x4] double -  transformation from robot base_link to obj
%--------------------------------------------------------------------------

    %% Local variables

    % Get robot handle
    r = optns{'rHandle'};

    %% Initialize variables     
    frameAdjustmentFlag = optns{"frameAdjustmentFlag"};     % Matlab's base_link does not match ros. Need adjustment.
    toolAdjustmentFlag  = optns{"toolAdjustmentFlag"};      % We have fingers but have not adjusted IKs for them. Transform from wrist to tips. 
    z_offset = optns{"z_offset"};                          % Instead of grasping model (i.e. can) from CoM, grasp from top.

    
    %% 1. Get Poses from matlab wrt to World
    disp('Get object pose from gazebo...');

    % Get (i) pose of robot's base_link and (i) pose of model (object) wrt gazebo's world origin
    W_T_R = get_model_pose('robot', optns);
    W_T_M = get_model_pose(model_name, optns);
    
    %% 2. Get Goal|Current Pose wrt to **MATLAB** base link in matlab format
    
    mat_W_T_R = ros2matlabPose(W_T_R, frameAdjustmentFlag, toolAdjustmentFlag, optns);
    mat_W_T_M = ros2matlabPose(W_T_M, frameAdjustmentFlag, toolAdjustmentFlag, optns); % Frame at junction with table
    
    % Change reference frame from world to robot's base_link
    mat_R_T_M =  mat_W_T_M/mat_W_T_R;  % OK to use inv for homogeneous transformation

    % Offset along +z_base_link to simulate knowing height of top of can.
    mat_R_T_M(3,4) = mat_R_T_M(3,4) + z_offset;
    
    
    %% 3. Modify orientation of robot pose to be a top-down pick (see tool0 vs base_link) w fingers aligned with matlab's y_world -axis
    fing_along_y = eul2tform([-pi/2 -pi 0]); % ZYX axis
    %fing_along_x = eul2tform([0 0 pi]); 

    mat_R_T_M(1:3,1:3) = fing_along_y(1:3,1:3);
            
    %% 4. Get Current base-to-gripper pose (ROS->MATLAB) 
    base = 'base_link';

    if get_robot_gripper_pose_flag
        end_effector = 'gripper_tip_link'; % When finger is properly modeled use 'gripper_tip_link'
    else
        end_effector = 'tool0';
    end

    % Get end-effector pose wrt to base via getTransform(tftree,targetframe,sourceframe), where sourceframe is the reference frame 
    try
        current_pose = getTransform(optns{"rHandle"}.tftree, end_effector, base);
    catch
        % Try again
        current_pose = getTransform(optns{"rHandle"}.tftree, end_effector, base);   
    end

    % Convert base-to-gripper pose to matlab format
    mat_R_T_G = ros2matlabPose(current_pose,...
                               frameAdjustmentFlag,...
                               toolAdjustmentFlag,...
                               optns);    
    
end