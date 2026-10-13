function [v, w, idx, st, lookPt] = controller(mode, pose, path, idx, P, st)
% CONTROLLER  Path tracking.
%   mode = 'purepursuit'  or  'pid'
%   pose = [x y theta],  path = N x 2 waypoints,  idx = progress along path
%   st   = memory for PID (integral, previous error)
%   returns commanded v [m/s], w [rad/s]

x = pose(1);  y = pose(2);  th = pose(3);
n = size(path,1);

% 1) closest waypoint (only search ahead, so we never go backwards)
win = idx:min(n, idx+20);
[~, k] = min(hypot(path(win,1)-x, path(win,2)-y));
idx = win(k);

% 2) look-ahead point: first waypoint at least Ld away
j = idx;
while j < n && hypot(path(j,1)-x, path(j,2)-y) < P.Ld
    j = j + 1;
end
lookPt = path(j,:);

% 3) angle from robot heading to the look-ahead point
dx = lookPt(1) - x;   dy = lookPt(2) - y;
alpha = atan2(dy, dx) - th;
alpha = atan2(sin(alpha), cos(alpha));        % wrap to [-pi, pi]
Ld_act = max(hypot(dx, dy), 1e-3);

% 4) speed: slow down for sharp turns and near the goal
dGoal = hypot(path(end,1)-x, path(end,2)-y);
v = P.v_max * max(cos(alpha), 0);
v = min(v, 0.8 * dGoal);

switch mode
    case 'purepursuit'
        % circle through robot and look-ahead point: curvature = 2 sin(alpha)/Ld
        w = v * 2 * sin(alpha) / Ld_act;
    case 'pid'
        e = alpha;                               % heading error
        st.int  = st.int + e * P.dt;
        de      = (e - st.prev) / P.dt;
        st.prev = e;
        w = P.Kp*e + P.Ki*st.int + P.Kd*de;
    otherwise
        error('controller: unknown mode %s', mode);
end

% target is behind us: stop and turn on the spot
if abs(alpha) > pi/2
    v = 0;
    w = 1.5 * alpha;
end
end
