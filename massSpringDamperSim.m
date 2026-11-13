m = 1; % kg
g = 9.81; % m/s^2
c = 2; % N*s/m (damper coefficient
k = 100; % N/m (spring coefficient chosen at random)
dt = 0.01; % s
dtb = 1;
dts = 0.001;
time = 5; % s

n = round(time/dt);
v = zeros(1, n);
t = zeros(1, n);
x = zeros(1, n);
x(1) = 0.1;

for i = 1:(n-1)
    a = (-k*x(i) - c*v(i))/m;
    x(i+1) = x(i) + v(i)*dt;
    v(i+1) = v(i) + a*dt;
    t(i+1) = t(i) + dt;
end

figure; plot(t, x);
xlabel('Time [s]'); ylabel('Position [m]');
title('Mass-spring-damper, Euler, dt = 0.01 s');
