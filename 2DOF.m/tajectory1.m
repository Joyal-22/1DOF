
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