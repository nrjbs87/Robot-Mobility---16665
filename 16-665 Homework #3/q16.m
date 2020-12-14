p = linspace(0,0.5,100);
v0 = linspace(0, 1, 100);

g = 9.81;

% hip height 
y0 = 1.5;

[P, V0] = meshgrid(p, v0);
xT_arr = -P + sqrt(g.*P.^2 + (V0.^2).*y0)/sqrt(g);
Z = zeros(100, 100) + 0.25;

surf(P, V0, xT_arr);
hold on
surf(P,V0,Z);
xlabel('P');
ylabel('V0');
zlabel('xT');