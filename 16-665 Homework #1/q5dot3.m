% define constants
clear 
Vx = 30;
m = 1573; 
Iz = 2873;
lf = 1.1;
lr = 1.58;
Caf = 80000;
Car = 80000;

% define LQR parameters
Q = diag([100000 1 5 .001]);
R = 1;

% define A, B1, B2 vectors
A = [
    [0, 1, 0, 0]; 
    [0, -(2*Caf + 2*Car)/(m*Vx), (2*Caf + 2*Car)/m, -(2*Caf*lf+2*Car*lr)/(m*Vx)];
    [0, 0, 0, 1];
    [0, -(2*Caf * lf - 2*Car*lr)/(Iz*Vx), (2*Caf*lf - 2*Car*lr)/Iz, -(2*Caf*lf^2+2*Car*lr^2)/(Iz*Vx)];
    ];
B1 = [
     0;
     (2*Caf)/m;
     0;
     (2*Caf *lf)/Iz;
     ];
 
B2 = [
     0;
     (-(2*Caf*lf - 2*Car * lr)/(m*Vx)) - Vx;
     0;
     (-(2*Caf*lf^2 + 2*Car*lr^2))/(Iz*Vx);
     ];

% solve for K using LQR and set initial positions 
K = lqr(A, B1, Q, R)
x0  = [0; 0; 0; 0;];

% set time steps
dt = 0.001;
final_step = 4;
tspan = 0:dt:final_step;

% instantiate error term and initial positions in 
% arrays holding values
err(:,1) = [0; 0; 0; 0];
t(1) = 0;
x_curr_arr = zeros(1, length(tspan));
y_curr_arr = zeros(1, length(tspan));
psides_arr = zeros(1, length(tspan));
x_curr_arr(1) = 0;
y_curr_arr(1) = -5;
x_des(1) = 0;
y_des(1) = -5;
psi = 0;
psidot_des = 0;

% booleans for turning 
turn_1_trigger = 0;
turn_2_trigger = 0;

% build x & y desired arrays for plotting desired path
for k = 1:length(tspan)
    psi_des = 0;
    if (x_des(k) <= 95) && (x_des(k) >= 5)
        psi_des = atan(5/90);
        psides_arr(k) = psi_des;
    end
    x_des(k+1) = x_des(k) + Vx * dt * cos(psi_des);
    y_des(k+1) = y_des(k) + Vx * dt * sin(psi_des);
end

% iterate through timesteps, depending on x and y pos, change psidot_des
% accordingly 
for k =  1:length(tspan)
    if (x_curr_arr(k) >= 5) && (x_curr_arr(k) <= 95)
        psi = atan(5/90);
    else
        psi = 0;
    end
    if (x_curr_arr(k) >= 5) && (turn_1_trigger == 0)
        psidot_des = psi/dt;
        turn_1_trigger = 1;
    elseif (y_curr_arr(k) >= 0) && (turn_2_trigger == 0)
        psi = 0;
        psidot_des = -atan(5/90)/dt;
        turn_2_trigger = 1;
    else 
        psidot_des = 0;
    end
    
  % update current position 
  x_curr_arr(k+1) = x_curr_arr(k) + (dt * Vx * cos(err(2,k) + psides_arr(k))); 
  y_curr_arr(k+1) = y_curr_arr(k) + (dt * Vx * sin(err(2,k) + psides_arr(k)));
  
  % use runga-kutta so solve ODE
  % please note this RK4 code was pulled from George Kantor's example code
  % from 16-642
  % https://canvas.cmu.edu/courses/19673/files?
  k1 = dt*ODEFunc(err(:,k),A, B1, K, B2, psidot_des);
  k2 = dt*ODEFunc(err(:,k)+k1/2, A, B1, K, B2, psidot_des);
  k3 = dt*ODEFunc(err(:,k)+k2/2, A, B1, K, B2, psidot_des);
  k4 = dt*ODEFunc(err(:,k)+k3,A, B1, K, B2, psidot_des);
  err(:,k+1) = err(:,k) + k1/6 + k2/3 + k3/3 + k4/6;
  t(k+1) = t(k) + dt;
  
end

hold on   

%plot actual vs desired trajectory
figure(1)
plot(x_des, y_des)
grid
plot(x_curr_arr, y_curr_arr)
yline(0, '--');
yline(-5, '--');
xline(95, '--');
xline(0, '--');
title('Actual(x,y) vs. Desired (x,y)');
xlabel('time (seconds)');
ylabel('y(t)');
l = legend({'Desired', 'Actual'}, 'Location', 'northwest');

%plot error terms 
% plot(t, err(1,:))
% plot(t, err(2,:))
% yline(.01, '--');
% yline(-.01, '--');
% yline(.0001, '--');
% yline(-.0001, '--');
% title('e1, e2 for lane change');
% xlabel('time (seconds)');
% ylabel('y(t)');
% l = legend({'e1', 'e2'}, 'Location', 'northwest');

% define ODE for KBM
function state_sol = ODEFunc(x, A, B1, K, B2, psidot_des)
state_sol = ((A-B1*K)*x) + B2*psidot_des;
end


