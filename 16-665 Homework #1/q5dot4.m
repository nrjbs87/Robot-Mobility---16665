% define constants
clear 
Vx = 60;
m = 1573; 
Iz = 2873;
lf = 1.1;
lr = 1.58;
Caf = 80000;
Car = 80000;

% define LQR parameters
Q = diag([150000 1 5000 1]);
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
x0  = [0; 0; 0; 0;];
K = lqr(A, B1, Q, R);
K

% set time steps
dt = 0.01;
final_step = 12;
tspan = 0:dt:final_step;

% instantiate error term and initial positions in 
% arrays holding values
err(:,1) = [0; 0; 0; 0];
t(1) = 0;
x_curr_arr = zeros(1, length(tspan));
y_curr_arr = zeros(1, length(tspan));
x_curr_arr(1) = 0;
y_curr_arr(1) = 0;
psi_arr = zeros(1, length(tspan));
x_des(1) = 0;
y_des(1) = 0;

% iterate through timesteps, depending on x and y pos, change psidot_des
% accordingly 
for k =  1:length(tspan)
    
    if tspan(k) < 1
        psidot_desired = 0;
    elseif (tspan(k) >= 1) && (tspan(k) < 6)
        psidot_desired = (Vx/1000);
    elseif (tspan(k) >= 6) && (tspan(k) < 7)
        psidot_desired = 0;
    elseif (tspan(k) >= 7) && (tspan(k) <= 12)
        psidot_desired = -(Vx/500);
        
    end
    
 % update current position 
  psi_arr(k+1) = psi_arr(k) + (psidot_desired * dt);
  x_des(k+1) = x_des(k) + dt * Vx * cos(psi_arr(k));
  y_des(k+1) = y_des(k) + dt * Vx * sin(psi_arr(k));
  
  x_curr_arr(k+1) = x_curr_arr(k) + (dt * Vx * cos(err(2,k) + psi_arr(k))); 
  y_curr_arr(k+1) = y_curr_arr(k) + (dt * Vx * sin(err(2,k) + psi_arr(k)));
  
  % use runga-kutta so solve ODE
  % please note this RK4 code was pulled from George Kantor's example code
  % from 16-642
  % https://canvas.cmu.edu/courses/19673/files?
  k1 = dt*ODEFunc(err(:,k),A, B1, K, B2, psidot_desired);
  k2 = dt*ODEFunc(err(:,k)+k1/2, A, B1, K, B2, psidot_desired);
  k3 = dt*ODEFunc(err(:,k)+k2/2, A, B1, K, B2, psidot_desired);
  k4 = dt*ODEFunc(err(:,k)+k3,A, B1, K, B2, psidot_desired);
  err(:,k+1) = err(:,k) + k1/6 + k2/3 + k3/3 + k4/6;
  t(k+1) = t(k) + dt;
end   
  hold on
  
%plot actual vs desired trajectory
plot(x_des, y_des);
plot(x_curr_arr, y_curr_arr)
grid;
title('Actual(x,y) vs. Desired (x,y)');
xlabel('time (seconds)');
ylabel('y(t)');
yline(0, '--');
l = legend({'Desired', 'Actual'}, 'Location', 'northwest');

%plot error terms 
plot(t, err(1,:))
plot(t, err(2,:))  
grid;
yline(.01, '--');
yline(-.01, '--');
title('e1, e2 for curve trajectory, V=60');
xlabel('time (seconds)');
ylabel('y(t)');
l = legend({'e1', 'e2'}, 'Location', 'northwest');

% define ODE for KBM
function state_sol = ODEFunc(x, A, B1, K, B2, psidot_des)
state_sol = ((A-B1*K)*x) + B2*psidot_des;
end


 