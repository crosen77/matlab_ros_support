%%% COMPLETED %%%

function resetWorld(optns)
%-------------------------------------------------------------------------- 
% resetWorld()
% Calls Gazebo service to reset the world
% Input: (dict) optns
% Output: None
%-------------------------------------------------------------------------- 
    disp('Resetting the world...');
    
    % TODO: 01 Get robot handle
    r = optns{"rHandle"};
    
    % TODO: 02 Create Empty Simulation message
    empty_msg = rosmessage(r.res_client);
    
    % TODO: 03 Call reset service
    call(r.res_client, empty_msg);

end