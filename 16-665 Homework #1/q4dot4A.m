%%%%%%%%%%%%%%%%%
%   Q 4A   %
%%%%%%%%%%%%%%%%%

% define constants
lr = 1.5;
lf = 1.5;
V = 10;
dt = 0.01;
tspan = 0:dt:30;

% define array of deltas 
delta_arr = [pi/6, pi/5, pi/4];

% define initial condition
init = [0, 0, 0];

hold on

for i = 1:length(delta_arr)
    x = init(1);
    y = init(2);
    psi = init(3);
    x_arr = zeros(1,length(tspan));
    y_arr = zeros(1,length(tspan));
    delta = delta_arr(i);

    for t =  1:length(tspan)
        % iterate tspan, use euler' to compute next
        % position
        x = x + (V*cos(psi) * dt);
        y = y + (V*sin(psi) * dt);
        psi = psi + ((V/(lr+lf) * tan(delta)) * dt);
        x_arr(t) = x;
        y_arr(t) = y; 
    end
    plot(x_arr, y_arr);
    title('Turning angles of of pi/6, pi/5, pi/4');
    xlabel('x(t)');
    ylabel('y(t)');
    legend('\delta_f = pi/6', '\delta_f = pi/5', '\delta_f = pi/4');
end
    
hold off


