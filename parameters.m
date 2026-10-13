function P = parameters()
% PARAMETERS  All robot, controller, map and simulation constants in one place.
% Change values here only - every other file reads from P.

%% ---------------- Robot geometry (small hospital delivery robot) ---------
P.r        = 0.075;   % wheel radius [m]  (15 cm diameter wheels)
P.L        = 0.40;    % wheelbase = distance between the two wheels [m]
P.body_len = 0.50;    % body length [m]  (for drawing + inertia)
P.body_wid = 0.45;    % body width  [m]

%% ---------------- Mass and inertia ---------------------------------------
P.m_empty   = 25;     % robot mass without medicine [kg]
P.m_payload = 10;     % medicine box / tray mass [kg]
P.m_wheel   = 1.0;    % mass of one wheel [kg]
P.Iw  = 0.5 * P.m_wheel * P.r^2;   % wheel inertia about its axle (solid disc) [kg m^2]
P.c_rr = 0.02;        % rolling-resistance coefficient (hard hospital floor)
P.g    = 9.81;        % gravity [m/s^2]
% Body inertia about vertical axis is computed in dynamics.m as
% I = m*(len^2 + wid^2)/12  (solid rectangular box), so it changes with payload.

%% ---------------- Limits (safety in a hospital corridor) -----------------
P.v_max     = 0.6;    % max linear speed [m/s]  (slow walking pace)
P.w_max     = 1.2;    % max angular speed [rad/s]
P.a_max     = 0.4;    % max linear acceleration [m/s^2] (so medicine doesn't slide)
P.alpha_max = 1.5;    % max angular acceleration [rad/s^2]
P.wheel_max = 10;     % max wheel speed [rad/s] (= 0.75 m/s at the rim)

%% ---------------- Controller ---------------------------------------------
P.Ld       = 0.7;     % Pure Pursuit look-ahead distance [m]
P.goal_tol = 0.15;    % "arrived" if closer than this to the goal [m]
P.Kp = 2.0;  P.Ki = 0.0;  P.Kd = 0.1;   % PID heading-controller gains (comparison)

%% ---------------- Map / planner ------------------------------------------
P.res     = 0.25;     % grid cell size [m]
P.inflate = 0.30;     % grow walls by this much so the robot body never touches them [m]
P.ward    = 'B';      % destination ward: 'A', 'B' or 'C'

%% ---------------- Simulation ---------------------------------------------
P.dt        = 0.05;   % time step [s]  (20 Hz)
P.t_deliver = 3;      % time spent stopped at the ward [s]
P.anim_skip = 2;      % redraw every N steps (bigger = faster animation)
end
