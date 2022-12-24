function err = rel_error(a, b)
a_ = reshape(a, [numel(a), 1]);
b_ = reshape(b, [numel(a), 1]);
err = abs(a_(a_ ~= 0) - b_(a_ ~= 0)) ./ abs(a_(a_ ~= 0));
end