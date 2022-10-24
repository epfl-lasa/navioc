function playsample(sample, mdp_data)
n = mdp_data.n_ped;
if n > 7
	C = repmat(colororder, [ceil(n/7), 1]);
else
	C = colororder;
end
C = C(1:n, :);
states = [sample.s; sample.states];
Px = states(:, 1:2:mdp_data.udims);
Py = states(:, 2:2:mdp_data.udims);
x_limits = [min(Px, [], 'all') - 0.2, max(Px, [], 'all') + 0.2];
y_limits = [min(Py, [], 'all') - 0.2, max(Py, [], 'all') + 0.2];
tmp_handles = [];
for i = 1:size(Px, 1)
	X = Px(i, :)';
	Y = Py(i, :)';
	
	%scatter(X, Y, 20, C)
	delete(tmp_handles)
	hold on
	if mod(i, 1) == 0
		w0 = i/size(Px, 1);
		w1 = 1 - w0;
		for j = 1:n
			plotcircle(X(j), Y(j), 0.2, C(j, :)*w0 + w1);
		end
	end	
	tmp_handles = [];
	for j = 1:n
		h = plotcircle(X(j), Y(j), 0.2, C(j, :));
		tmp_handles = [tmp_handles, h];
	end
	
	xlim(x_limits)
	ylim(y_limits)
	daspect([1, 1, 1])
	pause(0.05)
end


function h = plotcircle(x, y, r, c)
h = rectangle('Position', [x - r, y - r, 2*r, 2*r], ...
	'Curvature',[1, 1], 'FaceColor', c, 'EdgeColor', c);
