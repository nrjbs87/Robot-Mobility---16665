%% Introduction
% Main function for 16-665 Air Mobility project
% Last updated: October 2019

% Usage: main takes in a question number and executes all necessary code to
% construct a trajectory, plan a path, simulate a quadrotor, and control
% the model. Possible arguments: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9. THE
% TAS WILL BE RUNNING YOUR CODE SO PLEASE KEEP THIS MAIN FUNCTION CALL 
% STRUCTURE AT A MINIMUM.

% Requirements: no additional packages are needed to run this program as
% is. 

% Version: this framework was made using MATLAB R2018b. Should function
% as expected with most recent versions of MATLAB.

function [] = main(question)

if question == 2 || question == 3
    
    clc
    close all;
    
    %% Set up quadrotor physical parameters

    params = struct(...
        'mass',                   0.770, ...
        'gravity',                9.80665, ...
        'arm_length',           0.1103, ...
        'motor_spread_angle',     0.925, ...
        'thrust_coefficient',     8.07e-9, ...
        'moment_scale',           1.3719e-10, ...
        'motor_constant',        36.5, ...
        'rpm_min',             3000, ...
        'rpm_max',            20000, ...
        'inertia',            diag([0.0033 0.0033 0.005]),...
        'COM_vertical_offset',                0.05);

    %% Get the waypoints for this specific question
  
    [waypoints, waypoint_times] = lookup_waypoints(question);
    % waypoints are of the form [x, y, z, yaw]
    % waypoint_times are the seconds you should be at each respective waypoint,
    % make sure the simulation parameters below allow you to get to all points

    %% Set the simualation parameters

    time_initial = 0; 
    time_final = 10;
    time_step = 0.005; % sec
    % 0.005 sec is a reasonable time step for this system

    time_vec = time_initial:time_step:time_final;
    max_iter = length(time_vec);

    %% Create the state vector
    state = zeros(16,1);

    % Populate the state vector with the first waypoint (assumes that robot is
    % at the first waypoint at the initial time.
    state(1)  = waypoints(1,1); %x
    state(2)  = waypoints(2,1); %y
    state(3)  = waypoints(3,1); %z
    state(4)  =  0;        %xdot
    state(5)  =  0;        %ydot
    state(6)  =  0;        %zdot
    state(7) =   0;         %phi
    state(8) =   0;         %theta
    state(9) =   waypoints(4,1); %psi
    state(10) =  0;         %phidot 
    state(11) =  0;         %thetadot
    state(12) =  0;         %psidot
    state(13:16) =  0;      %rpm

    %% Create a trajectory consisting of desired state at each time step

    % Some aspects of this state we can plan in advance, some will be filled
    % in during the loop
    trajectory_matrix = trajectory_planner(waypoints, max_iter, waypoint_times, time_step);
    % [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];

    %% Create a matrix to hold the actual state at each time step
    actual_state_matrix = zeros(15, max_iter);  
    % [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];
    actual_state_matrix(:,1) = vertcat(state(1:12), 0, 0, 0);

    %% Create a matrix to hold the actual desired state at each time step

    % Need to store the actual desired state for acc, omega dot and omega as
    % it will be updated by the controller
    actual_desired_state_matrix = zeros(15, max_iter);  

    %% Loop through the timesteps and update quadrotor
    for iter = 1:max_iter-1
        
        % convert current state to stuct for control functions
        current_state.pos = state(1:3);
        current_state.vel = state(4:6);
        current_state.rot = state(7:9);
        current_state.omega = state(10:12);
        current_state.rpm = state(13:16);

        % Get desired state from matrix, put into struct for control functions
        desired_state.pos = trajectory_matrix(1:3,iter);
        desired_state.vel = trajectory_matrix(4:6,iter);
        desired_state.rot = trajectory_matrix(7:9,iter);
        desired_state.omega = trajectory_matrix(10:12,iter);
        desired_state.acc = trajectory_matrix(13:15,iter);

        % Get desired acceleration from position controller
        [F, desired_state.acc] = position_controller(current_state, desired_state, params, question);

        % Computes desired pitch and roll angles
        [desired_state.rot, desired_state.omega] = attitude_planner(desired_state, params);

        % Get torques from attitude controller
        M = attitude_controller(current_state, desired_state, params, question);

        [F_actual, M_actual, rpm_motor_dot] = motor_model(F, M, current_state.rpm, params);

        % Get the change in state from the quadrotor dynamics
        timeint = time_vec(iter:iter+1);
        [tsave, xsave] = ode45(@(t,s) dynamics(params, s, F_actual, M_actual, rpm_motor_dot), timeint, state);
        state    = xsave(end, :)';
        acc  = (xsave(end,4:6)' - xsave(end-1,4:6)')/(tsave(end) - tsave(end-1));

        % Update desired state matrix
        actual_desired_state_matrix(1:3,iter+1) =  desired_state.pos;
        actual_desired_state_matrix(4:6, iter+1) = desired_state.vel;
        actual_desired_state_matrix(7:9, iter+1) = desired_state.rot;
        actual_desired_state_matrix(10:12, iter+1) = desired_state.omega;
        actual_desired_state_matrix(13:15, iter+1) = desired_state.acc;

        % Update actual state matrix
        actual_state_matrix(1:12, iter+1) = state(1:12);
        actual_state_matrix(13:15, iter+1) = acc;  

    end

    plot_quadrotor_errors(actual_state_matrix, actual_desired_state_matrix, time_vec)
end

if question == 4
    
    clc
    close all;
    

    %% Set up quadrotor physical parameters

    params = struct(...
        'mass',                   0.770, ...
        'gravity',                9.80665, ...
        'arm_length',           0.1103, ...
        'motor_spread_angle',     0.925, ...
        'thrust_coefficient',     8.07e-9, ...
        'moment_scale',           1.3719e-10, ...
        'motor_constant',        36.5, ...
        'rpm_min',             3000, ...
        'rpm_max',            20000, ...
        'inertia',            diag([0.0033 0.0033 0.005]),...
        'COM_vertical_offset',                0.05);

    time_initial = 0; 
    time_final = 10;
    time_step = 0.005; % sec

    %% Create the state vector
    state = zeros(16,1);
    mode = 40;
    [waypoints, waypoint_times] = lookup_waypoints(question, mode);
    temp_time_vec = time_initial:time_step:1;
    max_iter = length(temp_time_vec);
    % Populate the state vector with the first waypoint (assumes that robot is
    % at the first waypoint at the initial time.
    state(1)  = waypoints(1,1); %x
    state(2)  = waypoints(2,1); %y
    state(3)  = waypoints(3,1); %z
    state(4)  =  0;        %xdot
    state(5)  =  0;        %ydot
    state(6)  =  0;        %zdot
    state(7) =   0;         %phi
    state(8) =   0;         %theta
    state(9) =   waypoints(4,1); %psi
    state(10) =  0;         %phidot 
    state(11) =  0;         %thetadot
    state(12) =  0;         %psidot
    state(13:16) =  0;      %rpm

    actual_desired_state_matrix = zeros(15, max_iter); 

    actual_state_matrix = zeros(15, max_iter);  

    actual_state_matrix(:,1) = vertcat(state(1:12), 0, 0, 0);

    total_time = 0;
    threshold = .25;
    
    %idle
    trajectory_matrix = trajectory_planner(waypoints, max_iter, waypoint_times, time_step);
    [temp_actual_state_matrix, temp_actual_desired_state_matrix, state, time_elapsed] = update(state, trajectory_matrix, params, question, max_iter, temp_time_vec,  time_step, -999);
    actual_desired_state_matrix = temp_actual_desired_state_matrix;
    actual_state_matrix = temp_actual_state_matrix;
    total_time = total_time + time_elapsed;
    disp("idle complete")
    
    mode = 41; % takeoff
    temp_time_vec = (1+time_step):time_step:4;
    max_iter = length(temp_time_vec);
    [waypoints, waypoint_times] = lookup_waypoints(question, mode);
    trajectory_matrix = trajectory_planner(waypoints, max_iter, waypoint_times, time_step);
    [temp_actual_state_matrix, temp_actual_desired_state_matrix, state, time_elapsed] = update(state, trajectory_matrix, params, question, max_iter, temp_time_vec,  time_step, threshold);
    actual_desired_state_matrix = [actual_desired_state_matrix, temp_actual_desired_state_matrix];
    actual_state_matrix = [actual_state_matrix, temp_actual_state_matrix];
    total_time = total_time + time_elapsed;
    disp("take off complete")

    mode = 42; % hover1
    temp_time_vec = (4+time_step):time_step:5;
    max_iter = length(temp_time_vec);
    [waypoints, waypoint_times] = lookup_waypoints(question, mode);
    trajectory_matrix = trajectory_planner (waypoints, max_iter, waypoint_times, time_step);
    [temp_actual_state_matrix, temp_actual_desired_state_matrix, state, time_elapsed] = update(state, trajectory_matrix, params, question, max_iter, temp_time_vec, time_step, -999);
    actual_desired_state_matrix = [actual_desired_state_matrix, temp_actual_desired_state_matrix];
    actual_state_matrix = [actual_state_matrix, temp_actual_state_matrix];
    total_time = total_time + time_elapsed;
    disp("hover 1 complete")

    mode = 43; % track
    temp_time_vec = (5+time_step):time_step:8;
    max_iter = length(temp_time_vec);
    [waypoints, waypoint_times] = lookup_waypoints(question, mode);
    trajectory_matrix = trajectory_planner(waypoints, max_iter, waypoint_times, time_step);
    [temp_actual_state_matrix, temp_actual_desired_state_matrix, state, time_elapsed] = update(state , trajectory_matrix, params, question, max_iter, temp_time_vec, time_step, threshold);
    actual_desired_state_matrix = [actual_desired_state_matrix, temp_actual_desired_state_matrix];
    actual_state_matrix = [actual_state_matrix, temp_actual_state_matrix];
    total_time = total_time + time_elapsed;
    disp("tracking complete")
    
    mode = 44; % hover2
    temp_time_vec = (8+time_step):time_step:9;
    max_iter = length(temp_time_vec);
    [waypoints, waypoint_times] = lookup_waypoints(question, mode);
    trajectory_matrix = trajectory_planner(waypoints, max_iter, waypoint_times, time_step);
    [temp_actual_state_matrix, temp_actual_desired_state_matrix, state, time_elapsed] = update(state , trajectory_matrix, params, question, max_iter, temp_time_vec, time_step, -999);
    actual_desired_state_matrix = [actual_desired_state_matrix, temp_actual_desired_state_matrix];
    actual_state_matrix = [actual_state_matrix, temp_actual_state_matrix];
    total_time = total_time + time_elapsed;
    disp("hover 2 complete")
    
    mode = 45; % land
    temp_time_vec = (9+time_step):time_step:12;
    max_iter = length(temp_time_vec);
    [waypoints, waypoint_times] = lookup_waypoints(question, mode);
    trajectory_matrix = trajectory_planner(waypoints, max_iter, waypoint_times, time_step);
    [temp_actual_state_matrix, temp_actual_desired_state_matrix, state, time_elapsed] = update(state , trajectory_matrix, params, question, max_iter, temp_time_vec, time_step, -999);
    actual_desired_state_matrix = [actual_desired_state_matrix, temp_actual_desired_state_matrix];
    actual_state_matrix = [actual_state_matrix, temp_actual_state_matrix];
    total_time = total_time + time_elapsed;
    disp("land complete")
    
    % slightly hacked, not sure why I have less time steps
    time_vec = 0:time_step:total_time + (6*time_step);
%     length(actual_desired_state_matrix)
%     length(actual_state_matrix)
%     length(time_vec)

    plot_quadrotor_errors(actual_state_matrix, actual_desired_state_matrix, time_vec)
       

end

if question == 5
    
    clc
    close all;
    
    %% Set up quadrotor physical parameters

    params = struct(...
        'mass',                   0.770, ...
        'gravity',                9.80665, ...
        'arm_length',           0.1103, ...
        'motor_spread_angle',     0.925, ...
        'thrust_coefficient',     8.07e-9, ...
        'moment_scale',           1.3719e-10, ...
        'motor_constant',        36.5, ...
        'rpm_min',             3000, ...
        'rpm_max',            20000, ...
        'inertia',            diag([0.0033 0.0033 0.005]),...
        'COM_vertical_offset',                0.05);

    time_initial = 0; 
    time_final = 10;
    time_step = 0.005; % sec
    

    %% Create the state vector
    state = zeros(16,1);
    mode = 41;
    [waypoints, waypoint_times] = lookup_waypoints(question, mode);
    temp_time_vec = time_initial:time_step:3;
    max_iter = length(temp_time_vec);
    % Populate the state vector with the first waypoint (assumes that robot is
    % at the first waypoint at the initial time.
    state(1)  = waypoints(1,1); %x
    state(2)  = waypoints(2,1); %y
    state(3)  = waypoints(3,1); %z
    state(4)  =  0;        %xdot
    state(5)  =  0;        %ydot
    state(6)  =  0;        %zdot
    state(7) =   0;         %phi
    state(8) =   0;         %theta
    state(9) =   waypoints(4,1); %psi
    state(10) =  0;         %phidot 
    state(11) =  0;         %thetadot
    state(12) =  0;         %psidot
    state(13:16) =  0;      %rpm

    actual_desired_state_matrix = zeros(15, max_iter); 

    actual_state_matrix = zeros(15, max_iter);  

    actual_state_matrix(:,1) = vertcat(state(1:12), 0, 0, 0);

    total_time = 0;
    threshold = .001;
    
    % start from takeoff opposed to idle
    trajectory_matrix = trajectory_planner(waypoints, max_iter, waypoint_times, time_step);
    [temp_actual_state_matrix, temp_actual_desired_state_matrix, state, time_elapsed] = update(state, trajectory_matrix, params, question, max_iter, temp_time_vec,  time_step, threshold);
    actual_desired_state_matrix = temp_actual_desired_state_matrix;
    actual_state_matrix = temp_actual_state_matrix;
    total_time = total_time + time_elapsed;
    disp("take off complete")

    mode = 42; % hover1
    temp_time_vec = (3+time_step):time_step:4;
    max_iter = length(temp_time_vec);
    [waypoints, waypoint_times] = lookup_waypoints(question, mode);
    trajectory_matrix = trajectory_planner (waypoints, max_iter, waypoint_times, time_step);
    [temp_actual_state_matrix, temp_actual_desired_state_matrix, state, time_elapsed] = update(state, trajectory_matrix, params, question, max_iter, temp_time_vec, time_step, -999);
    actual_desired_state_matrix = [actual_desired_state_matrix, temp_actual_desired_state_matrix];
    actual_state_matrix = [actual_state_matrix, temp_actual_state_matrix];
    total_time = total_time + time_elapsed;
    disp("hover 1 complete")

    mode = 43; % track
    temp_time_vec = (4+time_step):time_step:15;
    max_iter = length(temp_time_vec);
    [waypoints, waypoint_times] = lookup_waypoints(question, mode);
    trajectory_matrix = trajectory_planner(waypoints, max_iter, waypoint_times, time_step);
    [temp_actual_state_matrix, temp_actual_desired_state_matrix, state, time_elapsed] = update(state , trajectory_matrix, params, question, max_iter, temp_time_vec, time_step, threshold);
    actual_desired_state_matrix = [actual_desired_state_matrix, temp_actual_desired_state_matrix];
    actual_state_matrix = [actual_state_matrix, temp_actual_state_matrix];
    total_time = total_time + time_elapsed;
    disp("tracking complete")
    
    % slightly hacked, not sure why I have less time steps
    time_vec = 0:time_step:total_time + (3*time_step);
    length(actual_desired_state_matrix)
    length(actual_state_matrix)
    length(time_vec)

    plot_quadrotor_errors(actual_state_matrix, actual_desired_state_matrix, time_vec)
       

end

end

function [actual_state_matrix, actual_desired_state_matrix, state, time_elapsed] = update(state, trajectory_matrix, params, question, max_iter, time_vec, time_step, threshold)
    
   % update the first index of the actual_state_matrix 
   % so it doesn't start from 0 in the graph
    actual_state_matrix(1:12, 1) = state(1:12);
    actual_desired_state_matrix(1:3,1) = trajectory_matrix(1:3,1);
    actual_desired_state_matrix(4:6,1) = trajectory_matrix(4:6,1);
    actual_desired_state_matrix(7:9,1) = trajectory_matrix(7:9,1);
    actual_desired_state_matrix(10:12,1) = trajectory_matrix(10:12,1);
    actual_desired_state_matrix(13:15,1) = trajectory_matrix(13:15,1);

    final_desired_state = trajectory_matrix(:, end);

    time_elapsed = 0;

    for iter = 1:max_iter-1

        % convert current state to stuct for control functions
        current_state.pos = state(1:3);
        current_state.vel = state(4:6);
        current_state.rot = state(7:9);
        current_state.omega = state(10:12);
        current_state.rpm = state(13:16);

        % Get desired state from matrix, put into struct for control functions
        desired_state.pos = trajectory_matrix(1:3,iter);
        desired_state.vel = trajectory_matrix(4:6,iter);
        desired_state.rot = trajectory_matrix(7:9,iter);
        desired_state.omega = trajectory_matrix(10:12,iter);
        desired_state.acc = trajectory_matrix(13:15,iter);

        % Get desired acceleration from position controller
        [F, desired_state.acc] = position_controller(current_state, desired_state, params, question);

        % Computes desired pitch and roll angles
        [desired_state.rot, desired_state.omega] = attitude_planner(desired_state, params);

        % Get torques from attitude controller
        M = attitude_controller(current_state, desired_state, params, question);

        [F_actual, M_actual, rpm_motor_dot] = motor_model(F, M, current_state.rpm, params);

        % Get the change in state from the quadrotor dynamics
        timeint = time_vec(iter:iter+1);
        [tsave, xsave] = ode45(@(t,s) dynamics(params, s, F_actual, M_actual, rpm_motor_dot), timeint, state);
        state    = xsave(end, :)';
        acc  = (xsave(end,4:6)' - xsave(end-1,4:6)')/(tsave(end) - tsave(end-1));

        % Update desired state matrix
        actual_desired_state_matrix(1:3,iter+1) =  desired_state.pos;
        actual_desired_state_matrix(4:6, iter+1) = desired_state.vel;
        actual_desired_state_matrix(7:9, iter+1) = desired_state.rot;
        actual_desired_state_matrix(10:12, iter+1) = desired_state.omega;
        actual_desired_state_matrix(13:15, iter+1) = desired_state.acc;

        % Update actual state matrix
        actual_state_matrix(1:12, iter+1) = state(1:12);
        actual_state_matrix(13:15, iter+1) = acc;  

        time_elapsed = time_elapsed + time_step;

        error = [final_desired_state(1) - state(1),
                 final_desired_state(2) - state(2),
                 final_desired_state(3) - state(3),
                 final_desired_state(9) - state(9)];
%        norm(error)
%        threshold
        if norm(error) < threshold 
            %disp("breaking")
            break;
        end
    end



end



