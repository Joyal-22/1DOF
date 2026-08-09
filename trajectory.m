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

