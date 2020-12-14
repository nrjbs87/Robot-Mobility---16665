function [rot, omega] = attitude_planner(desired_state, params)

% Input parameters
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
% Output parameters
%
%   rot: will be stored as desired_state.rot = [phi; theta; psi], 
%
%   omega: will be stored as desired_state.omega = [phidot; thetadot; psidot]
%
%************  ATTITUDE PLANNER ************************

% Write code here

g = params.gravity;
psi_des = desired_state.rot(3);
psi_des_dot = desired_state.omega(3);

rot = zeros(3,1);
rot = 1/g * [sin(psi_des), -cos(psi_des); cos(psi_des), sin(psi_des)] * [desired_state.acc(1); desired_state.acc(2)];
rot(3) = psi_des;


omega = zeros(3,1);
omega = 1/g * [cos(psi_des), sin(psi_des); -sin(psi_des), cos(psi_des)] * [desired_state.acc(1) * psi_des_dot; desired_state.acc(2) * psi_des_dot];
omega(3) = psi_des_dot;

end

