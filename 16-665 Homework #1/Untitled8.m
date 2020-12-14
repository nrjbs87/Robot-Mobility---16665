Wh = linspace(0,400,400);
A = 1;

Hc_arr = zeros(1, length(Wh));
Hs_arr = zeros(1, length(Wh));
for i = 1:length(Wh)
    
Hc = 69*A + .364*Wh(i);
Hs = 1*A + .577*Wh(i);
Hc_arr(i) = Hc;
Hs_arr(i) = Hs;
end

hold on
plot(Wh, Hc_arr)
plot(Wh, Hs_arr)

title('Clay vs. Sand Thrust');
xlabel('Weight');
ylabel('Soil Thurst');
l = legend({'Clay Thrust', 'Sand Thrust'}, 'Location', 'northwest');