clear all 
% define constants 
k = 20000;  % N/m
r = 0.05;   % meters
%N = 25;     % ratio 
% N should be 80
N = 30;     % ratio 
M = 80;     % kg
g = 9.81;   % m/s^2
J = .00054; %kgm^2
%J = 5.1e-4;
T_max = 1.56;
%T_max = 1.49;


% define time steps
t_end = 20;
t_start = 0;
timestep = .001;
tspan = t_start : timestep : t_end;

% define initial state vector
state = zeros(4,1);
state(1) = 1; %y
state(2) = 0; %ydot
state(3) = 0; %theta
state(4) = 0; %thetadot

% define gains (y)
Kp_outer = 30;
Kd_outer = 0;
Ki_outer = 0;

Kp_inner = 500;
Kd_inner = 50;

% Kp_outer = 1000;
% Kp_outer = 30;
% Kd_outer = 0;
% Ki_outer = 0;
% 
% Kp_inner = 1000;
% Kd_inner = 100;

y_des = 0.7;

ey = y_des - state(1);
etheta = state(3);
ey_i = 0;

states = zeros(4, length(tspan));
states(:,1)= state;
Tm_arr = zeros(1, length(tspan));
y0 = M * g / k + 1;

for i = 2:length(tspan)
    %%%%%%%%%%%%
    %outer loop%
    %%%%%%%%%%%%
    ey_last = ey;
    ey = y_des - state(1);
    %ey_dot = (ey - ey_last)/timestep;
    ey_dot = -state(2);
    %ey_i = ey_i + ey;
    Fs_des = M*((Kp_outer * ey) + (Kd_outer * ey_dot) + Ki_outer*(ey_i) + g);
    %Fs_des = M*((-Kp_outer * ey) + (-Kd_outer * ey_dot) - Ki_outer*(ey_i) + g);
    %Fs_des = ((-Kp_outer * ey) + (-Kd_outer * ey_dot) - Ki_outer*(ey_i) + M*g);
   
    %theta_M_des = N/r * ((Fs_des/k) + state(1) - y_des);
    theta_M_des = N/(r*k) * ((Fs_des) - k* (y0 - state(1)));
    
    %%%%%%%%%%%%
    %inner loop%
    %%%%%%%%%%%%
    etheta_last = etheta;
    etheta = theta_M_des - state(3);
    %etheta_dot = (etheta - etheta_last)/timestep;
    etheta_dot = -state(4);
    Tm = J*(Kp_inner * etheta + Kd_inner * etheta_dot) + Fs_des*r/N;
    %Tm = J*(-Kp_inner * etheta - Kd_inner * etheta_dot); %+ Fs_des*r/N;
    %Tm = (-Kp_inner * etheta - Kd_inner * etheta_dot); 
    if Tm > T_max
       Tm = T_max;
    end
%     if Tm < -T_max
%        Tm = -T_max;
%     end
    Fs = k*(y0 - state(1) + r/N * state(3));
    %Fs = k*(y_des - state(1) + (r/N)*state(3));
    
    %%%%%%%%%%%%%%
    %state update%
    %%%%%%%%%%%%%%
    state_dot = zeros(4,1);
    state_dot(1) = state(2);
    state_dot(2) = Fs/M - g;
    state_dot(3) = state(4); 
    state_dot(4) = (Tm - (Fs * (r/N))) / J;
   
    %states(:, i) = states(:, i-1) + timestep*state_dot;
    %state = states(:,i);
    
    state = state + state_dot * timestep;
    states(:,i) = state;
    
    Tm_arr(i) = Tm;
    
end
figure(1);
plot(tspan, states(1,:));
yline(y_des, '--');
figure(2);
plot(tspan, states(2,:));


% figure(2);
% plot(tspan, Tm_arr);
