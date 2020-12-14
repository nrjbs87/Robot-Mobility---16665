%%%%%%%%%%%%%%%%%
%   Q 4C   %
%%%%%%%%%%%%%%%%%

% define constants
lr = 1.5;
lf = 1.5;
V = 10;
dt = 0.01;
tspan = 0:dt:30;

% define array of deltas 
delta_arr = [pi/12, pi/11, pi/10];

% define initial condition
init = [0, 0, 0];

hold on

for i = 1:length(delta_arr)
    amp = delta_arr(i);
    x = init(1);
    y = init(2);
    psi = init(3);
    x_arr = zeros(1,length(tspan));
    y_arr = zeros(1,length(tspan));

    for t =  1:length(tspan)
        delta = amp * square(tspan(t));
        x = x + (V*cos(psi) * dt);
        y = y + (V*sin(psi) * dt);
        psi = psi + ((V/(lr+lf) * tan(delta)) * dt);
        x_arr(t) = x;
        y_arr(t) = y; 
    end
    plot(x_arr, y_arr);
    title('Turning angles of of pi/12, pi/11, pi/10, V = 10');
    xlabel('x(t)');
    ylabel('y(t)');
    l = legend({'\delta_f = pi/12', '\delta_f = pi/11', '\delta_f = pi/10'}, 'Location', 'northwest');
end
    
hold off




