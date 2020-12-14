% define theta & gravity
theta = 30;
g = 9.81;

% create vector to loop through for lf and h
lf = linspace(0.25, 1.75, 7);
h = linspace(0.5, 2, 7);

% for each h test every lf and build deaccel vector
for i = 1:length(h)
    deaccel_arr = zeros(1,length(lf));
    for j = 1:length(lf)
        decel = g/h(i)*(cosd(theta)*lf(j) - h(i)*sind(theta));
        deaccel_arr(j) = decel;       
    end
    
    % plot lf vs. deaccel
    hold on;
    plot(lf, deaccel_arr);
    t = title('lf vs. deceleration', 'FontSize', 24);
    xlabel('lf (m)', 'FontSize', 24);
    ylabel('deacceleration (m/s^2)', 'FontSize', 24);
    l = legend({'h = 0.5', 'h = 0.75', 'h = 1.0', 'h = 1.25', 'h = 1.5', 'h = 1.75', 'h = 2.0'}, 'Location', 'northwest');
    l.FontSize = 40;
    
end
hold off;


