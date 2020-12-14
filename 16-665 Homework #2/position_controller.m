function [F, acc] = position_controller(current_state, desired_state, params, question)

% Input parameters
% 
%   current_state: The current state of the robot with the following fields:
%   current_state.pos = [x; y; z], 
%   current_state.vel = [x_dot; y_dot; z_dot],
%   current_state.rot = [phi; theta; psi], 
%   current_state.omega = [phidot; thetadot; psidot]
%   current_state.rpm = [w1; w2; w3; w4];
%
%   desired_state: The desired states are:
%   desired_state.pos = [x; y; z], 
%   desired_state.vel = [x_dot; y_dot; z_dot],
%   desired_state.rot = [phi; theta; psi], 
%   desired_state.omega = [phidot; thetadot; psidot]
%   desired_state.acc = [xdotdot; ydotdot; zdotdot];
%
%   params: Quadcopter parameters
%
%   question: Question number
%
% Output parameters
%
%   F: u1 or thrust
%
%   acc: will be stored as desired_state.acc = [xdotdot; ydotdot; zdotdot]
%
%************  POSITION CONTROLLER ************************

% Example PD gains
% Kp1 = 17;
% Kd1 = 6.6;
Kp1 = 20;
Kd1 = 8;

% Kp2 = 17;
% Kd2 = 6.6;
Kp2 = 20;
Kd2 = 8;

% Kp3 = 20;
% Kd3 = 9;
Kp3 = 18;
Kd3 = 9;

Kp = diag([Kp1, Kp2, Kp3]);
Kd = diag([Kd1, Kd2, Kd3]);

% define constants we need
m = params.mass;
g = params.gravity;

% define b vector
% x and y are zero 
% gravity is negative in the interial frame 
b = [0; 0; 1;];

% calc error
edd_xyz = (-Kp * (current_state.pos - desired_state.pos)) - (Kd * (current_state.vel - desired_state.vel));

% acc now defines error + desired_acc
acc = edd_xyz + desired_state.acc;
% calc forces in z-direction of body frame 
F = m * b' * ((g * [0;0;1;]) + acc);

end
