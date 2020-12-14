tspan = 0:0.01:10;
y0 = [0; 0; 0; 0];


[t,y] = ode45(@func, tspan,y0); 

plot(t,y)

function xdot = func(t, x)

    xdot = x.^2;
    %xdot(3,3) = 5532;
    %xdot = xdot * x;
end