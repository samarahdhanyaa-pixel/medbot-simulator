function path = planner(M, start, goal)
% PLANNER  A* search on the inflated occupancy grid (8-connected).
%   start, goal : [x y] in metres
%   path        : N x 2 list of waypoints [x y] in metres
%
% (Toolbox equivalent, if you want to mention it in the viva:
%    map = binaryOccupancyMap(M.infl, 1/M.res);
%    planner = plannerAStarGrid(map);  -- Navigation Toolbox
%  We write A* ourselves so every line can be explained.)

G = M.infl;  res = M.res;
[ny, nx] = size(G);
toRC = @(p) [min(ny, floor(p(2)/res)+1), min(nx, floor(p(1)/res)+1)];  % [row col]

s  = toRC(start);   gl = toRC(goal);
si = sub2ind([ny nx], s(1),  s(2));
gi = sub2ind([ny nx], gl(1), gl(2));
if G(si) || G(gi), error('Start or goal is inside a wall'); end

N = ny*nx;
g      = inf(N,1);      % cost from start
f      = inf(N,1);      % g + heuristic
parent = zeros(N,1);
isOpen = false(N,1);  isClosed = false(N,1);
h = @(r,c) hypot(r - gl(1), c - gl(2));     % straight-line heuristic (cells)

g(si) = 0;  f(si) = h(s(1), s(2));  isOpen(si) = true;
moves = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];

cur = si;
while any(isOpen)
    fo = f;  fo(~isOpen) = inf;
    [~, cur] = min(fo);                     % open node with lowest f
    if cur == gi, break; end
    isOpen(cur) = false;  isClosed(cur) = true;
    [r, c] = ind2sub([ny nx], cur);

    for k = 1:8
        rr = r + moves(k,1);  cc = c + moves(k,2);
        if rr < 1 || rr > ny || cc < 1 || cc > nx, continue; end
        if G(rr,cc), continue; end
        % diagonal move: don't cut corners of walls
        if k > 4 && (G(r,cc) || G(rr,c)), continue; end
        ni = sub2ind([ny nx], rr, cc);
        if isClosed(ni), continue; end
        gNew = g(cur) + hypot(moves(k,1), moves(k,2));
        if gNew < g(ni)
            g(ni) = gNew;  parent(ni) = cur;
            f(ni) = gNew + h(rr, cc);
            isOpen(ni) = true;
        end
    end
end
if cur ~= gi, error('A*: no path found'); end

% walk back from goal to start
list = gi;
while parent(list(1)) ~= 0
    list = [parent(list(1)); list]; %#ok<AGROW>
end
[rr, cc] = ind2sub([ny nx], list);
path = [(cc - 0.5)*res, (rr - 0.5)*res];   % cell centres in metres
path(1,:) = start;  path(end,:) = goal;

% Densify: resample the path every 5 cm so it is smooth
% and the tracking error is measured properly
d = [0; cumsum(hypot(diff(path(:,1)), diff(path(:,2))))];  % distance along path
s = 0:0.05:d(end);
if s(end) < d(end), s = [s d(end)]; end
path = [interp1(d, path(:,1), s)', interp1(d, path(:,2), s)'];
end
