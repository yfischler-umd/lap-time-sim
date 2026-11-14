%MR = spring travel / wheel travel
%the spring force is k_s·MR·x.
%the force at the wheel is k_s·MR²·x
%wheel rate: k_w = k_s · MR²
%damper at the wheel: c_w = c · MR² (same logic)
%z_s for the body, z_u for the wheel, z_r for the road
%The suspension force is F = k_w(z_u − z_s) + c_w(ż_u − ż_s)
%Body: m_s · z̈_s = k_w(z_u − z_s) + c_w(ż_u − ż_s)
%Wheel: m_u · z̈_u = −k_w(z_u − z_s) − c_w(ż_u − ż_s) + k_t(z_r − z_u)


p = params();
h = @(t) 0.01 * (t >= 0.1); 
t = [0, 5];

function res = quarter_car(p, zr, tspan)
% p      : struct with ms, mu, ks, c, kt, g
% zr     : function handle, road height (m) as a function of time
% tspan  : [t_start t_end] in seconds
% out    : struct with t, zs, zu, body_acc, travel, Fz

y0 = [0; 0; 0; 0];                 % [zs; zs_dot; zu; zu_dot] at rest

opts = odeset('MaxStep', 1e-3);    % small steps so road inputs aren't skipped
[t, y] = ode45(@(t,y) rhs(t, y, p, zr), tspan, y0, opts);

zs = y(:,1);  vs = y(:,2);  zu = y(:,3);  vu = y(:,4);

% derived quantities
body_acc = (-p.ks_f*(zs-zu) - p.c*(vs-vu)) / p.m_sf;       % m/s^2
travel   = zs - zu;                                    % m
Fz       = (p.m_sf + p.m_u)*p.g + p.kt*(zr(t) - zu);      % N (zr must accept a vector)
Fz       = max(Fz, 0);                                 % tire can't pull on the road

res = struct('t',t, 'zs',zs, 'zu',zu, 'body_acc',body_acc, 'travel',travel, 'Fz',Fz);
end

function dy = rhs(t, y, p, zr)
zs = y(1); vs = y(2); zu = y(3); vu = y(4);
F_susp = p.ks_f*(zs-zu) + p.c*(vs-vu);        % force the spring+damper put on the unsprung mass
dy = [ vs;
    -F_susp / p.m_sf;
    vu;
    (F_susp + p.kt*(zr(t)-zu)) / p.m_u ];
end

tr = quarter_car(p, h, t);
figure; plot(tr.t, tr.zs, tr.t, tr.zu);
xlabel('Time [s]'); ylabel('Height [m]');
title('Quarter Car Model'); legend('Sprung Mass Relative Height','Unsprung Mass Relative height');
grid on

figure; plot( tr.t, h(tr.t));
xlabel('Time [s]'); ylabel('Height [m]');
title('Quarter Car Model'); legend('Road height');
grid on

figure; plot( tr.t, tr.Fz);
xlabel('Time [s]'); ylabel('Force [N]');
title('Quarter Car Model'); legend('Tire Load');
grid on