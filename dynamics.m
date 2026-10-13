function out = dynamics(mode, a, v, m, P)
% DYNAMICS  Newton-Euler model of a differential-drive robot.
%
%  out = dynamics('inverse', [v_dot w_dot], v, m, P) -> motor torques [tauL tauR] [N m]
%  out = dynamics('forward', [tauL tauR],   v, m, P) -> accelerations [v_dot w_dot]
%
%  m = total mass (robot + payload). Call with P.m_empty or P.m_empty+P.m_payload
%  to compare the empty and loaded robot.

r = P.r;  L = P.L;

% Effective mass: body mass + the two spinning wheels "feel" like extra mass
M_eff = m + 2 * P.Iw / r^2;

% Body inertia about the vertical axis (solid box) - grows with payload
I_body = m * (P.body_len^2 + P.body_wid^2) / 12;
% Effective inertia: wheels also have to speed up / slow down when turning
I_eff  = I_body + P.Iw * L^2 / (2 * r^2);

% Rolling resistance always opposes motion (zero when stopped)
F_rr = P.c_rr * m * P.g * sign(v);

switch mode
    case 'inverse'
        v_dot = a(1);  w_dot = a(2);
        Fx = M_eff * v_dot + F_rr;     % total forward force needed  [N]
        Mz = I_eff * w_dot;            % total turning moment needed [N m]
        % Fx = FR + FL,  Mz = (FR - FL)*L/2,  tau = F*r
        tauR = r/2 * (Fx + 2*Mz/L);
        tauL = r/2 * (Fx - 2*Mz/L);
        out = [tauL, tauR];

    case 'forward'
        tauL = a(1);  tauR = a(2);
        FL = tauL / r;   FR = tauR / r;   % wheel torque -> push force on ground
        Fx = FR + FL - F_rr;
        Mz = (FR - FL) * L/2;
        out = [Fx / M_eff, Mz / I_eff];

    otherwise
        error('dynamics: unknown mode %s', mode);
end
end
