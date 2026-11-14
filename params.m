function p = params()
p.g   = 9.81;                            % m/s^2
p.rho = 1.225;                           % air density, kg/m^3
p.wd = 0.512;                            % static front weight distribution, %
p.m   = 268.9;                           % car + driver mass, kg
p.m_u = 6.80;                            %unsprung mass per corner, kg
p.m_sf = (p.m*p.wd-2*p.m_u)/2;           %sprung mass per front corner, kg
p.mr = 0.850;                            %motion ratio
p.k_spring = 43.78*1000;                 %front spring rate, N/m
p.ks_f = p.k_spring*p.mr^2;                % suspension spring, N/m
p.kt = 111.7*1000;                       %tire stiffness, N/m
zeta = 0.3;                              % damping ratio chosen arbitrarily
p.c  = 2 * zeta * sqrt(p.ks_f * p.m_sf); % damper coefficient front, N*s/m
p.mu_tire  = 1.4;                        % tire friction coefficient (placeholder)
p.ClA = 5.253;                           % downforce coefficient * area, m^2
p.R_skid = 9.125;                        % skidpad driving radius, m
end