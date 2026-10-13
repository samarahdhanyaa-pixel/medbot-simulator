function out = kinematics(mode, a, b, P)
% KINEMATICS  Differential-drive kinematics.
%
%  out = kinematics('forward', wL, wR, P)    wheel speeds [rad/s] -> [v w]
%  out = kinematics('inverse', v,  w,  P)    [v w] -> wheel speeds [wL wR]
%  out = kinematics('pose',  pose, [v w], P) integrate one time step -> new [x y theta]
%
%  v = linear speed of robot centre [m/s],  w = angular speed (omega) [rad/s]

r = P.r;  L = P.L;

switch mode
    case 'forward'
        wL = a;  wR = b;
        vL = r * wL;              % rim speed of left wheel  [m/s]
        vR = r * wR;              % rim speed of right wheel [m/s]
        v  = (vR + vL) / 2;       % centre moves at the average speed
        w  = (vR - vL) / L;       % speed difference makes it turn
        out = [v, w];

    case 'inverse'
        v = a;  w = b;
        vR = v + w * L/2;         % right wheel is L/2 to the right of centre
        vL = v - w * L/2;
        out = [vL / r, vR / r];   % convert m/s -> rad/s

    case 'pose'
        x = a(1);  y = a(2);  th = a(3);
        v = b(1);  w = b(2);
        x_dot  = v * cos(th);     % unicycle model
        y_dot  = v * sin(th);
        th_dot = w;
        x  = x  + x_dot  * P.dt;  % Euler integration
        y  = y  + y_dot  * P.dt;
        th = th + th_dot * P.dt;
        th = atan2(sin(th), cos(th));   % keep angle in [-pi, pi]
        out = [x, y, th];

    otherwise
        error('kinematics: unknown mode %s', mode);
end
end
