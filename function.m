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

function [q1, dq1, ddq1] = J1(t)
% Cubic trajectory generator for Joint 1 - evaluated at current time t (from Clock)

T1  = 5;                    % Motion duration
q01 = deg2rad(30);         % initial starting angle for Joint 1
qf1 = deg2rad(90);         % final target angle for Joint 1

% Clamp t so trajectory holds steady after T1
tt = min(t, T1);

% --- Position Calculation ---
q1   = q01 + (qf1 - q01) * (3*(tt/T1)^2 - 2*(tt/T1)^3);

% --- Velocity Calculation (First Derivative) ---
dq1  = (qf1 - q01) * (6*tt/T1^2 - 6*(tt^2)/T1^3);

% --- Acceleration Calculation (Second Derivative) ---
ddq1 = (qf1 - q01) * (6/T1^2 - 12*tt/T1^3);
end 