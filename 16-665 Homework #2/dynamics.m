function [state_dot] = dynamics(params, state, F, M, rpm_motor_dot)
% Input parameters
% 
%   state: current state, will be using ode45 to update
%
%   F, M: actual force and moment from motor model
%
%   rpm_motor_dot: actual change in RPM from motor model
% 
%   params: Quadcopter parameters
%
%   question: Question number
%
% Output parameters
%
%   state_dot: change in state
%
%************  DYNAMICS ************************

% Write code here

state_dot = zeros(16,1);

state_dot(1:3) = state(4:6);
state_dot(7:9) = state(10:12);

phi = state(7);
theta = state(8);
psi = state(9);

m = params.mass;
g = params.gravity;
I = params.inertia;

R = zeros(6,1);
Fe = [F * (cos(phi) * cos(psi) * sin(theta) + sin(phi) * sin(psi));
      F * (cos(phi) * sin(theta) * sin(psi) - cos(psi) * sin(phi));
      F * cos(theta) * cos(phi) - m*g];
  
R(1:3) = Fe;
R(4:6) = M;

mI = m * diag([1,1,1]);
Z = zeros(3,3);
%omega = state(7:9);
omega = state(10:12);

L = [mI, Z; Z, I];
invL = [1/m 0 0 0 0 0;
        0 1/m 0 0 0 0;
        0 0 1/m 0 0 0;
        0 0 0 1/I(1,1) 0 0;
        0 0 0 0 1/I(2,2) 0;
        0 0 0 0 0 1/I(3,3);];

wI = zeros(6, 1);
wI(4:6) = cross(omega, I * omega); 

acc = invL * (R - wI);


state_dot(4:6) = acc(1:3);
state_dot(10:12) = acc(4:6);
state_dot(13:16) = rpm_motor_dot;

end