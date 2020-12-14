clear all 
% define constants 
k = 20000;     % N/m
r = 0.05;      % meters
N = 40;        % gear ratio 
M = 80;        % kg
g = 9.81;      % m/s^2
J = 5.3e-4;    % kgm^2
T_max = 1.363; % Nm

% define time steps
t_end = 15;
t_start = 0;
timestep = .001;
tspan = t_start : timestep : t_end;

% define gains (y)
Kp_outer = 10;
Kd_outer = 0;

% define gains (theta)
Kp_inner = 50;
Kd_inner = 10;

% define y_des values
y_des_vec  = [0.7 0.8 0.9];

for i = 1:length(y_des_vec)
    % define initial state vector
    state = zeros(4,1);
    state(1) = 1; %y
    state(2) = 0; %ydot
    state(3) = 0; %theta
    state(4) = 0; %thetadot
    
    % compute initial y_des and errors
    y_des = y_des_vec(i);
    ey = y_des - state(1);
    etheta = state(3);

    states = zeros(4, length(tspan));
    Tm_arr = zeros(1, length(tspan));
    states(:,1)= state;
    y0 = M * g / k + 1;
    
    Fs = 0;
    
    for i = 2:length(tspan)
        
        %%%%%%%%%%%%
        %outer loop%
        %%%%%%%%%%%%
        ey = y_des - state(1);
        ey_dot = -state(2);
        Fs_des = M*((Kp_outer * ey) + (Kd_outer * ey_dot) + g);
        Fs = k*(y0 - state(1) + r/N * state(3));
        
        %%%%%%%%%%%%
        %inner loop%
        %%%%%%%%%%%%
        theta_M_des = N/(r*k) * ((Fs_des) - k* (y0 - state(1)));
        etheta = theta_M_des - state(3);
        etheta_dot = -state(4);
        Tm = J*(Kp_inner * etheta + Kd_inner * etheta_dot) + Fs_des*r/N;

        % scale motor torque as not to exceed T_max
        if Tm > T_max
           Tm = T_max;
        end

        %%%%%%%%%%%%%%
        %state update%
        %%%%%%%%%%%%%%
        state_dot = zeros(4,1);
        state_dot(1) = state(2);
        state_dot(2) = Fs/M - g;
        state_dot(3) = state(4); 
        state_dot(4) = (Tm - (Fs * (r/N))) / J;

        % update state using Euler's
        state = state + state_dot * timestep;
        states(:,i) = state;

        Tm_arr(i) = Tm;

    end
    hold on 
    figure(1);
    plot(tspan, states(1,:));
    title('y-position vs. time');
    ylabel('y-position');
    xlabel('time');
    legend('y_{des} = 0.7', 'y_{des} = 0.8', 'y_{des} = 0.9', 'fontsize', 15);
    
    figure(2);
    plot(tspan, states(2,:));
    title('theta vs. time');
    ylabel('theta');
    xlabel('time');
    
    figure(3);
    plot(tspan, Tm_arr);
    title('motor torque vs. time');
    ylabel('motor torque');
    xlabel('time');
    legend('y_{des} = 0.7', 'y_{des} = 0.8', 'y_{des} = 0.9', 'fontsize', 15);
    
end

figure(1);
yline(y_des_vec(1), '--');
yline(y_des_vec(2), '--');
yline(y_des_vec(3), '--');

figure(3);
yline(T_max, '--');



