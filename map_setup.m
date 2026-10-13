function M = map_setup(P)
% MAP_SETUP  Builds a 20 m x 12 m hospital floor as an occupancy grid.
%   true = wall (occupied), false = free.
%
%   +-----------+-----------+---------+
%   |  Ward A   |  Ward B   | Ward C  |   y = 7.5 .. 12
%   +---door----+---door----+--door---+
%   |            CORRIDOR             |   y = 5 .. 7.5
%   +--door--+------------------------+
%   |Pharmacy|    (other rooms)       |   y = 0 .. 5
%   +--------+------------------------+

res = P.res;  W = 20;  H = 12;
nx = round(W/res);  ny = round(H/res);

% centre coordinate of every cell (rows = y, columns = x)
[X, Y] = meshgrid(((1:nx) - 0.5)*res, ((1:ny) - 0.5)*res);
occ = false(ny, nx);
box = @(x1,x2,y1,y2) (X >= x1 & X <= x2 & Y >= y1 & Y <= y2);

% outer walls
occ = occ | box(0,W,0,0.25) | box(0,W,H-0.25,H) | box(0,0.25,0,H) | box(W-0.25,W,0,H);

% corridor bottom wall (pharmacy door at x = 2..4)
occ = occ | box(0,2,4.8,5.2) | box(4,W,4.8,5.2);
% pharmacy right wall
occ = occ | box(4.8,5.2,0,5);

% corridor top wall (ward doors at x = 6..8, 12..14, 17..19)
occ = occ | box(0,6,7.3,7.7) | box(8,12,7.3,7.7) | box(14,17,7.3,7.7) | box(19,W,7.3,7.7);
% walls between wards
occ = occ | box(9.3,9.7,7.5,H) | box(15.3,15.7,7.5,H);

% ---- inflate walls by robot radius (configuration space) ----
k = ceil(P.inflate / res);
infl = occ;
for dr = -k:k
    for dc = -k:k
        if dr^2 + dc^2 <= k^2
            sh = false(ny, nx);
            rs = max(1,1+dr):min(ny,ny+dr);
            cs = max(1,1+dc):min(nx,nx+dc);
            sh(rs, cs) = occ(rs-dr, cs-dc);   % shifted copy of the walls
            infl = infl | sh;
        end
    end
end

M.occ = occ;  M.infl = infl;  M.res = res;  M.W = W;  M.H = H;
M.pharmacy = [3, 2.5];
M.wards.A  = [7, 10];
M.wards.B  = [13, 10];
M.wards.C  = [18, 10];
end
