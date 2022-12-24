function err = abs_error(a, b)
d = reshape(a - b, [numel(a), 1]);
err = abs(d);
err = reshape(err, size(a));
end