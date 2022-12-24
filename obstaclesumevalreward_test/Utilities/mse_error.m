function err = mse_error(a, b)
err = sum((a-b).^2, 'all')./numel(a);
end
