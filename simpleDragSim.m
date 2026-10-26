m = 1;
g = 9.81;
c = 0.1;
dt = 0.01;
time = 5;

n = round(time/dt);
v = zeros(1, n);
t = zeros(1, n);

for i = 1:(n-1)
    a = g - (c/m)*power(v(i), 2);
    v(i+1) = v(i) + a*dt;
    t(i+1) = t(i) + dt;
end

disp(v(n))
v_term = (sqrt(m*g/c));

figure; plot(t, v, t, v_term*ones(size(t)), '--');
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('Ball drop velocity with regard to time'); legend('Simulated velocity','Terminal velocity');