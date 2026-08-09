function [q, dq, ddq] = J1(t)
% Cubic trajectory generator - evaluated at current time t (from Clock)

T  = 5;                    % motion duration (s)
q0 = deg2rad(30);         % start angle
qf = deg2rad(120);        % end angle

% Clamp t so trajectory holds steady after T
tt = min(t, T);

% Cubic trajectory equation
q   = q0 + (qf - q0) * (3*(tt/T)^2 - 2*(tt/T)^3);

dq  = (qf - q0) * (6*tt/T^2 - 6*(tt^2)/T^3);

ddq = (qf - q0) * (6/T^2 - 12*tt/T^3);
end

function [t, tau, tau_g, tau_i, tau_f] = torque(q, dq, ddq)
% Torque computation for a single DOF robotic arm joint

% --- Robot Parameters ---
L = 0.4;                   % link length (m)
m = 2;                     % link mass (kg)
r = L/2;                   % center of mass distance from joint (m)
g = 9.81;                  % gravity acceleration (m/s^2)
B = 0.02;                  % viscous friction coefficient (N*m*s/rad)

% --- Motion duration ---
T = 5;                     % total trajectory duration (s)

% --- Ensure column vectors ---
q   = q(:);
dq  = dq(:);
ddq = ddq(:);

% --- Time vector ---
t = linspace(0, T, length(q));

% --- Inertia ---
I = (1/3) * m * L^2;       % moment of inertia for uniform link about one end

% --- Torque components ---
tau_i = I * ddq;           % inertial torque component

tau_g = m * g * r * cos(q);% gravitational torque component

tau_f = B * dq;            % friction torque component

% --- Total torque ---
tau = tau_i + tau_g + tau_f;
end