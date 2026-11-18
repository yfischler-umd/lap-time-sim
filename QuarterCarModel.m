p = params();
h = @(t) 0.01 * (t >= 0.1); 
t = [0, 5];

function res = quarter_car(p, h, tspan)
% p      : struct with ms, mu, ks, c, kt, g
% zr     : function handle, road height (m) as a function of time
% tspan  : [t_start t_end] in seconds
% out    : struct with t, zs, zu, body_acc, travel, Fz

y0 = [0; 0; 0; 0];                 % [zs; zs_dot; zu; zu_dot] at rest

opts = odeset('MaxStep', 1e-3);    % small steps so road inputs aren't skipped
[t, y] = ode45(@(t,y) rhs(t, y, p, h), tspan, y0, opts);

zs = y(:,1);  vs = y(:,2);  zu = y(:,3);  vu = y(:,4);

% derived quantities
body_acc = -(p.ks_f*(zs-zu)+p.c*(vs-vu))/p.m_sf;         % m/s^2
travel = zs - zu;                                        % m
Fz = (p.m_sf + p.m_u)*p.g + p.kt*(h(t) - zu);            % N
Fz = max(Fz, 0);                                         % tire can't pull on the road so no negative force

res = struct('t',t, 'zs',zs, 'zu',zu, 'body_acc',body_acc, 'travel',travel, 'Fz',Fz);
end

function dy = rhs(t, y, p, h)
zs = y(1); vs = y(2); zu = y(3); vu = y(4);
F_susp = p.ks_f*(zs-zu) + p.c*(vs-vu);        % force the spring+damper put on the unsprung mass
dy = [ vs; -F_susp / p.m_sf; vu; (F_susp + p.kt*(h(t)-zu)) / p.m_u ];
end

tr = quarter_car(p, h, t);
figure; plot(tr.t, tr.zs, tr.t, tr.zu, tr.t, tr.travel);
xlabel('Time [s]'); ylabel('Height [m]');
title('Quarter Car Model'); legend('Sprung Mass Relative Height','Unsprung Mass Relative height', 'Suspension Travel');
grid on

figure; plot( tr.t, h(tr.t));
xlabel('Time [s]'); ylabel('Height [m]');
title('Quarter Car Model'); legend('Road height');
grid on

figure; plot( tr.t, tr.Fz);
xlabel('Time [s]'); ylabel('Force [N]');
title('Quarter Car Model'); legend('Tire Load');
grid on
