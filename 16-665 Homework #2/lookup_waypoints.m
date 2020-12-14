function [waypoints, waypoint_times] = lookup_waypoints(question, mode)

% Input parameters
%
%   question: which question of the project we are on 
%      Possible arguments for question: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9, 10
%
% Output parameters
%
%   waypoints: of the form [x; y; z; yaw]
% 
%   waypoint_times: [1 x n] vector of times where n is the number of waypoings, 
%   represents the seconds you should be at each respective waypoint
%
%************ LOOKUP WAYPOINTS ************************

% Write code here

%Sample waypoints for hover trajectory

if nargin == 1
    
    if question == 2
        waypoints = [0 0.1 0.2 0.3; 0 0 0 0; 0.5 0.5 0.5 0.5; 0 0 0 0];
        waypoint_times = [0 2 4 6];

    end

    if question == 3 
        waypoint_vel = linspace(0.02, -0.02, 200);
        waypoints_zero = linspace(0, 0, 200);
        waypoint_times = linspace(0, 10, 200);
        waypoints_z = cumtrapz(waypoint_vel);
        waypoints = [waypoints_zero; waypoints_zero; waypoints_z; waypoints_zero];
    end

elseif nargin == 2

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %   edit expected trajectory for 4 here   %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if question == 4
        
        x_des = 0;
        y_des = 0;
        z_des = 1.0;
        z_des_drop = 0.3;
        psi_des_drop = 0;

    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %   edit expected trajectory for 5 here   %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if question == 5

        x_des = 0;
        y_des = 0;
        z_des = 0.5;
        z_des_drop = 0.1;
        psi_des_drop = 0.262;

    end


    if mode == 40
    % idle     
        waypoints_zero = linspace(0, 0, 200);
        waypoints = [waypoints_zero; waypoints_zero; waypoints_zero; waypoints_zero;]; 
        waypoint_times = linspace(0, 1, 200);
    end

    if mode == 41
    % take off 
        waypoints_zero = linspace(0, 0, 200);
        waypoints_z = linspace(0, z_des, 200);
        waypoints = [waypoints_zero; waypoints_zero; waypoints_z; waypoints_zero;]; 
        waypoint_times = linspace(0, 2, 200);
    end

    if mode == 42
    % hover 1 from 2-4   
        waypoints_zero = linspace(0, 0, 200);
        waypoints_z = (zeros(200,1) + z_des)';
        waypoints = [waypoints_zero; waypoints_zero; waypoints_z; waypoints_zero;]; 
        waypoint_times = linspace(0, 2, 200);
    end

    if mode == 43
    % tracking for x,y
        waypoints_zero = linspace(0, 0, 200);
        waypoints_x = linspace(0, x_des, 200);
        waypoints_y = linspace(0, y_des, 200);
        waypoints_z = linspace(z_des_drop, z_des_drop, 200);
        waypoints_psi = linspace(psi_des_drop, psi_des_drop, 200);
        %waypoints_z = (zeros(200,1) + 1)';
        waypoints = [waypoints_x; waypoints_y; waypoints_z; waypoints_psi;]; 
        waypoint_times = linspace(0, 2, 200);
    end

    if mode == 44
    % hover 2 
        waypoints_zero = linspace(0, 0, 200);
        waypoints_x = (zeros(200,1) + x_des)';
        waypoints_y = (zeros(200,1) + y_des)';
        waypoints_z = (zeros(200,1) + z_des_drop)';
        waypoints_psi = linspace(psi_des_drop, psi_des_drop, 200);
        waypoints = [waypoints_x; waypoints_y; waypoints_z; waypoints_psi;]; 
        waypoint_times = linspace(0, 2, 200);
    end

    if mode == 45
    % land 
        waypoints_zero = linspace(0, 0, 200);
        waypoints_psi = linspace(psi_des_drop, psi_des_drop, 200);
        waypoints_x = (zeros(200,1) + x_des)';
        waypoints_y = (zeros(200,1) + y_des)';
        waypoints_z = linspace(z_des_drop, 0, 200);
        waypoints = [waypoints_x; waypoints_y; waypoints_z; waypoints_psi;]; 
        waypoint_times = linspace(0, 1, 200);

    end
    


end

end
