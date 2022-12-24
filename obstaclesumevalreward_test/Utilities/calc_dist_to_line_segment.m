function [dist, pc, h] = calc_dist_to_line_segment(points, walls)
% This function is no longer used.

walls_size = size(walls, 1);
points_size = size(points, 2);

x1 = walls(:, 1)';
x2 = walls(:, 2)';
y1 = walls(:, 3)';
y2 = walls(:, 4)';

a = [x1; y1];
c = [x2; y2];
a_minus_c = a - c; 
a_minus_c_rotated = -[0, -1; 1, 0] * a_minus_c; 

a_minus_c_combined = zeros(2, walls_size*2);
a_minus_c_combined(:, 1:2:end) = a_minus_c;
a_minus_c_combined(:, 2:2:end) = a_minus_c_rotated;

d = reshape(a_minus_c_combined, 2, 2, []);
d = squeeze(num2cell(d, [1, 2]));
d = blkdiag(d{:});

parameters = d\(repmat(points, walls_size, 1)-repmat(c(:), 1, points_size));
alpha = parameters(1:2:end, :);
alpha = alpha.';
alpha_vec = reshape(alpha, 1, []);
h = kron(a_minus_c, ones(1, points_size)) * diag(alpha_vec) +... 
    kron(c, ones(1, points_size));
% h = pagetranspose(permute(reshape(h, walls_size, points_size, []),...
%     [3, 1, 2]));
% h = permute(pagetranspose(permute(reshape(h, walls_size, points_size, []),...
%     [3, 1, 2])), [3, 2, 1]);


pc = h;
[row_idx, col_idx] = find(alpha<=0);
pc(:, alpha_vec<=0) = c(:, col_idx); 
[row_idx, col_idx] = find(alpha>=1);
pc(:, alpha_vec>=1) = a(:, col_idx); 
pc = pagetranspose(permute(reshape(pc, walls_size, points_size, []),...
     [3, 1, 2]));
dist = vecnorm(reshape(points, 2, [], points_size)-pc, 2, 1); 

pc = permute(pc, [3, 2, 1]);
dist = permute(dist, [3, 2, 1]);

end

% walls_size = size(walls, 1);
% points_size = size(points, 2);
% pc = zeros(walls_size, points_size, 2);
% dist = zeros(walls_size, points_size);
% 
% for i = 1:size(walls, 1)
%     a = [walls(i, 1); walls(i, 3)];
%     c = [walls(i, 2); walls(i, 4)];
%     a_minus_c = a - c;
%     d = [a_minus_c, -[0, -1; 1, 0]*a_minus_c];
%     parameters = d\(points-c);
%     alpha = parameters(1, :);
%     h = a_minus_c * alpha + c;
%     pc(i, alpha<=0, :) = c;
%     pc(i, alpha>=1, :) = a;
%     pc(i, alpha>0 && alpha<1, :) = h;
%     dist(i, :) = vecnorm(points-pc(i, :, :), 2, 1);
% end