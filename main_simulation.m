%% MAIN_SIMULATION  Hospital medicine delivery robot - run this file.
% Pharmacy -> Ward (stop 3 s) -> Pharmacy, with live animation and plots.
clear; clc; close all;

P = parameters();
M = map_setup(P);

%% ---------------- Plan both legs with A* ----------------------------------
start = M.pharmacy;
goal  = M.wards.(P.ward);
path_go   = planner(M, start, goal);
path_back = planner(M, goal, start);
L_plan = sum(hypot(diff(path_go(:,1)),   diff(path_go(:,2)))) + ...
         sum(hypot(diff(path_back(:,1)), diff(path_back(:,2))));

%% ---------------- Draw the hospital map -----------------------------------
figure('Name','Hospital delivery robot','Color','w');
ax = axes; hold(ax,'on');
img = ones(size(M.occ));  img(M.infl) = 0.85;  img(M.occ) = 0;   % white/grey/black
imagesc(ax, [M.res/2, M.W-M.res/2], [M.res/2, M.H-M.res/2], img);
colormap(ax, gray); caxis(ax,[0 1]); axis(ax,'equal'); axis(ax,'xy');
xlim(ax,[0 M.W]); ylim(ax,[0 M.H]); xlabel('x [m]'); ylabel('y [m]');
text(1.0,1.0,'PHARMACY','FontWeight','bold');
text(5.5,11.3,'WARD A','FontWeight','bold');
text(11.5,11.3,'WARD B','FontWeight','bold');
text(16.5,11.3,'WARD C','FontWeight','bold');
text(8.5,6.2,'CORRIDOR','Color',[0.5 0.5 0.5]);
text(10,2.5,'(other rooms)','Color',[0.5 0.5 0.5]);

plot(path_go(:,1),   path_go(:,2),   'b--', 'LineWidth',1.2);
plot(path_back(:,1), path_back(:,2), 'g--', 'LineWidth',1.2);
H.traj  = plot(nan, nan, 'r-', 'LineWidth', 1.8);          % actual trajectory
H.body  = patch(nan, nan, [0.2 0.6 1], 'EdgeColor','k');   % robot body
H.arrow = plot(nan, nan, 'k-', 'LineWidth', 2.5);           % heading arrow
H.look  = plot(nan, nan, 'mo', 'MarkerFaceColor','m');      % look-ahead point
H.title = title(ax, '');
legend(ax, {'planned (go)','planned (return)','actual'}, 'Location','southeast');

%% ---------------- Simulate -------------------------------------------------
% LOG columns: 1 t | 2 x | 3 y | 4 theta | 5 v | 6 w | 7 wL | 8 wR |
%              9 v_dot | 10 w_dot | 11 tracking error | 12 phase(1 go,2 wait,3 back)
LOG  = zeros(0, 12);
pose = [start, pi/2];          % start facing the pharmacy door (north)
v_prev = 0;  w_prev = 0;  t = 0;

% Leg 1: pharmacy -> ward
[pose, v_prev, w_prev, t, LOG] = run_leg(path_go, 'purepursuit', pose, ...
                                         v_prev, w_prev, t, LOG, P, H, 1);
t_arrive = t;

% Delivery: stand still for 3 s
v_prev = 0;  w_prev = 0;
for i = 1:round(P.t_deliver / P.dt)
    t = t + P.dt;
    LOG(end+1,:) = [t, pose, 0, 0, 0, 0, 0, 0, 0, 2]; %#ok<SAGROW>
    set(H.title, 'String', sprintf('Delivering medicine to Ward %s ...  t = %.1f s', P.ward, t));
    drawnow; pause(P.dt);
end

% Leg 2: ward -> pharmacy
[pose, v_prev, w_prev, t, LOG] = run_leg(path_back, 'purepursuit', pose, ...
                                         v_prev, w_prev, t, LOG, P, H, 3);
set(H.title, 'String', sprintf('Back at pharmacy. Total time = %.1f s', t));

%% ---------------- Results --------------------------------------------------
T = LOG(:,1);  X = LOG(:,2);  Y = LOG(:,3);
V = LOG(:,5);  W = LOG(:,6);  WL = LOG(:,7);  WR = LOG(:,8);
A = LOG(:,9);  ALPHA = LOG(:,10);  ERR = LOG(:,11);
moving = LOG(:,12) ~= 2;
L_act = sum(abs(V)) * P.dt;

% motor torques for SAME motion, empty vs loaded
tauE = zeros(numel(T),2);  tauF = zeros(numel(T),2);
for i = 1:numel(T)
    tauE(i,:) = dynamics('inverse', [A(i) ALPHA(i)], V(i), P.m_empty, P);
    tauF(i,:) = dynamics('inverse', [A(i) ALPHA(i)], V(i), P.m_empty + P.m_payload, P);
end

fprintf('\n===== DELIVERY SUMMARY (Ward %s) =====\n', P.ward);
fprintf('Pharmacy -> ward time      : %6.1f s\n', t_arrive);
fprintf('Total round-trip time      : %6.1f s  (incl. %d s delivery stop)\n', t, P.t_deliver);
fprintf('Planned path length (both) : %6.2f m\n', L_plan);
fprintf('Actual distance travelled  : %6.2f m\n', L_act);
fprintf('Tracking error RMS / max   : %6.3f / %.3f m\n', sqrt(mean(ERR(moving).^2)), max(ERR));
fprintf('Peak wheel torque empty    : %6.3f N m\n', max(abs(tauE(:))));
fprintf('Peak wheel torque loaded   : %6.3f N m\n', max(abs(tauF(:))));
fprintf('RMS  wheel torque empty    : %6.3f N m\n', sqrt(mean(tauE(:).^2)));
fprintf('RMS  wheel torque loaded   : %6.3f N m\n', sqrt(mean(tauF(:).^2)));

figure('Name','Results','Color','w');
subplot(2,2,1); hold on; axis equal; box on;
contour(((1:size(M.occ,2))-0.5)*M.res, ((1:size(M.occ,1))-0.5)*M.res, double(M.occ), [0.5 0.5], 'k');
plot(path_go(:,1), path_go(:,2), 'b--', path_back(:,1), path_back(:,2), 'g--');
plot(X, Y, 'r-', 'LineWidth', 1.2);
title('Planned vs actual trajectory'); xlabel('x [m]'); ylabel('y [m]');
legend('walls','planned go','planned return','actual','Location','southeast');

subplot(2,2,2);
plot(T, ERR, 'LineWidth', 1.2); grid on;
title('Tracking error vs time'); xlabel('t [s]'); ylabel('distance to path [m]');

subplot(2,2,3);
plot(T, WL, T, WR, 'LineWidth', 1.2); grid on;
title('Wheel angular velocities'); xlabel('t [s]'); ylabel('[rad/s]'); legend('\omega_L','\omega_R');

subplot(2,2,4);
yyaxis left;  plot(T, V, 'LineWidth', 1.2); ylabel('v [m/s]');
yyaxis right; plot(T, W, 'LineWidth', 1.2); ylabel('\omega [rad/s]');
grid on; title('Linear and angular velocity'); xlabel('t [s]');

figure('Name','Motor torque: empty vs loaded','Color','w');
subplot(2,1,1);
plot(T, tauE(:,1), 'b', T, tauF(:,1), 'r', 'LineWidth', 1.1); grid on;
title('Left motor torque'); ylabel('\tau_L [N m]'); legend('empty (25 kg)','loaded (35 kg)');
subplot(2,1,2);
plot(T, tauE(:,2), 'b', T, tauF(:,2), 'r', 'LineWidth', 1.1); grid on;
title('Right motor torque'); ylabel('\tau_R [N m]'); xlabel('t [s]');
legend('empty (25 kg)','loaded (35 kg)');

%% ---------------- Optional: Pure Pursuit vs PID heading (leg 1 only) -------
[~,~,~,~,LOGpid] = run_leg(path_go, 'pid', [start, pi/2], 0, 0, 0, zeros(0,12), P, [], 1);
LOGpp = LOG(LOG(:,12)==1, :);
figure('Name','Pure Pursuit vs PID','Color','w');
subplot(1,2,1); hold on; axis equal; box on;
contour(((1:size(M.occ,2))-0.5)*M.res, ((1:size(M.occ,1))-0.5)*M.res, double(M.occ), [0.5 0.5], 'k');
plot(path_go(:,1), path_go(:,2), 'k--');
plot(LOGpp(:,2), LOGpp(:,3), 'r', LOGpid(:,2), LOGpid(:,3), 'm', 'LineWidth', 1.2);
legend('walls','planned','Pure Pursuit','PID'); title('Trajectory (pharmacy \rightarrow ward)');
subplot(1,2,2);
plot(LOGpp(:,1), LOGpp(:,11), 'r', LOGpid(:,1), LOGpid(:,11), 'm', 'LineWidth', 1.2); grid on;
legend('Pure Pursuit','PID'); xlabel('t [s]'); ylabel('error [m]'); title('Tracking error');
fprintf('Leg-1 RMS error  Pure Pursuit: %.3f m | PID: %.3f m\n', ...
        sqrt(mean(LOGpp(:,11).^2)), sqrt(mean(LOGpid(:,11).^2)));


%% ======================= LOCAL FUNCTIONS ===================================
function [pose, v_prev, w_prev, t, LOG] = run_leg(path, mode, pose, v_prev, w_prev, t, LOG, P, H, phase)
% Drive along one path until the goal is reached.
idx = 1;  st.int = 0;  st.prev = 0;  step = 0;  t_max = t + 300;
while hypot(pose(1)-path(end,1), pose(2)-path(end,2)) > P.goal_tol && t < t_max
    % controller -> desired v, w
    [v_cmd, w_cmd, idx, st, look] = controller(mode, pose, path, idx, P, st);

    % speed limits
    v_cmd = max(-P.v_max, min(P.v_max, v_cmd));
    w_cmd = max(-P.w_max, min(P.w_max, w_cmd));
    % acceleration limits (change per step is limited)
    v = v_prev + max(-P.a_max*P.dt,     min(P.a_max*P.dt,     v_cmd - v_prev));
    w = w_prev + max(-P.alpha_max*P.dt, min(P.alpha_max*P.dt, w_cmd - w_prev));

    % inverse kinematics: body speed -> wheel speeds (+ wheel speed limit)
    wh = kinematics('inverse', v, w, P);
    s = max(abs(wh)) / P.wheel_max;
    if s > 1, wh = wh / s; end

    % forward kinematics: wheel speeds -> what the robot actually does
    vw = kinematics('forward', wh(1), wh(2), P);
    v = vw(1);  w = vw(2);

    a_lin = (v - v_prev) / P.dt;       % accelerations (for dynamics/torque)
    a_ang = (w - w_prev) / P.dt;

    pose = kinematics('pose', pose, [v w], P);    % move the robot
    t = t + P.dt;
    err = min(hypot(path(:,1)-pose(1), path(:,2)-pose(2)));

    LOG(end+1,:) = [t, pose, v, w, wh, a_lin, a_ang, err, phase]; %#ok<AGROW>
    v_prev = v;  w_prev = w;

    % animation
    if ~isempty(H) && mod(step, P.anim_skip) == 0
        draw_robot(H, pose, P);
        set(H.traj, 'XData', LOG(:,2), 'YData', LOG(:,3));
        set(H.look, 'XData', look(1), 'YData', look(2));
        if phase == 1, txt = 'Going to ward'; else, txt = 'Returning to pharmacy'; end
        set(H.title, 'String', sprintf('%s   t = %.1f s   v = %.2f m/s', txt, t, v));
        drawnow; pause(P.dt * P.anim_skip);   % real-time speed
    end
    step = step + 1;
end
end

function draw_robot(H, pose, P)
% Robot = rotated rectangle + arrow showing heading.
x = pose(1);  y = pose(2);  th = pose(3);
c = [ P.body_len/2,  P.body_wid/2;
     -P.body_len/2,  P.body_wid/2;
     -P.body_len/2, -P.body_wid/2;
      P.body_len/2, -P.body_wid/2];
R = [cos(th) -sin(th); sin(th) cos(th)];     % rotation matrix
cw = (R * c')';                               % rotate corners
set(H.body,  'XData', cw(:,1) + x, 'YData', cw(:,2) + y);
set(H.arrow, 'XData', [x, x + 0.5*cos(th)], 'YData', [y, y + 0.5*sin(th)]);
end
