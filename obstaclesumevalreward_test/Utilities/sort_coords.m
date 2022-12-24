function [coords_sorted] = sort_coords(coords, flag_ccw)
x = coords(1, :);
y = coords(2, :);
cx = mean(x);
cy = mean(y);
angles = atan2(y-cy, x-cx);
[~, ids] = sort(angles, 'ascend');
if flag_ccw
    coords_sorted = coords(:, ids);
else
    coords_sorted = coords(:, flip(ids));
end
end