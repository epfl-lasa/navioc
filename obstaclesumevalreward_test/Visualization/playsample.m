function fig = playsample(sample, mdp_data, no_video, x_limits, y_limits, walls)
%pause(0.01) % switch to newest figure
% fig = figure('Position', [300, 600, 2000, 600]);
fig = figure;
ax = axes(fig);
if nargin < 3
	no_video = false;
end
if ~no_video
	pause(0.1)
end
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
Vx = states(:, mdp_data.udims + (1:2:mdp_data.udims));
Vy = states(:, mdp_data.udims + (2:2:mdp_data.udims));
if nargin < 5
	x_limits = [min_(Px) - 0.2, max_(Px) + 0.2];
	y_limits = [min_(Py) - 0.2, max_(Py) + 0.2];
end
if nargin >= 6
	hold(ax, 'on')
	for w = 1:size(walls, 1)
		plot(ax, walls(w, 1:2), walls(w, 3:4), "k")
	end
	hold(ax, 'off')
end
tmp_handles = [];
for i = 1:size(Px, 1)
	X = Px(i, :)';
	Y = Py(i, :)';
	U = Vx(i, :)';
	V = Vy(i, :)';
	
	%scatter(X, Y, 20, C)
	delete(tmp_handles)
	hold(ax, 'on')
	if mod(i, 1) == 0
		w0 = i/size(Px, 1);
		w1 = 1 - w0;
		for j = 1:n
			plotcircle(ax, X(j), Y(j), 0.2, C(j, :)*w0 + w1);
		end
	end	
	tmp_handles = [];
	for j = 1:n
		if mdp_data.type(j) == 1
			h = plotcircle(ax, X(j), Y(j), 0.2, C(j, :), 'k');
		elseif mdp_data.type(j) == 3
			h = plotcircle(ax, X(j), Y(j), 0.2, C(j, :), 'r');
		else
			h = plotcircle(ax, X(j), Y(j), 0.2, C(j, :));
		end
		tmp_handles = [tmp_handles, h];
	end
	UV = reshape(mdp_data.v_des, [2, n])';
	h = quiver(ax, X, Y, UV(:, 1), UV(:, 2), 0, 'Color', 'k');
	tmp_handles = [tmp_handles, h];

	h = quiver(ax, X, Y, U, V, 0, 'Color', 'r');
	tmp_handles = [tmp_handles, h];
	
	xlim(ax, x_limits)
	ylim(ax, y_limits)
	daspect(ax, [1, 1, 1])
	if ~no_video
		if isfield(mdp_data, 'sampling_time')
			pause(mdp_data.sampling_time)
		else
			pause(0.05)
		end
		if isobject(fig) & ~isgraphics(fig)
			break
		end
	end
end
if ~no_video
	pause(0.05)
end

function h = plotcircle(ax, x, y, r, c1, c2)
if nargin == 5
	c2 = c1;
end
h = rectangle(ax, 'Position', [x - r, y - r, 2*r, 2*r], ...
	'Curvature',[1, 1], 'FaceColor', c1, 'EdgeColor', c2);

function C = colororder
C = [
	0 0.4470 0.7410	
	0.8500 0.3250 0.0980
	0.9290 0.6940 0.1250
	0.4940 0.1840 0.5560
	0.4660 0.6740 0.1880
	0.3010 0.7450 0.9330
	0.6350 0.0780 0.1840
];

function y = min_(x)
y = min(reshape(x, [numel(x), 1]));

function y = max_(x)
y = max(reshape(x, [numel(x), 1]));