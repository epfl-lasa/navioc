function playsample(sample, mdp_data, no_video)
%pause(0.01) % switch to newest figure
figure
pause(0.1)
if nargin < 3
	no_video = false;
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
x_limits = [min_(Px) - 0.2, max_(Px) + 0.2];
y_limits = [min_(Py) - 0.2, max_(Py) + 0.2];
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
	if ~no_video
		pause(0.05)
	end
end
pause(0.05)

function h = plotcircle(x, y, r, c)
h = rectangle('Position', [x - r, y - r, 2*r, 2*r], ...
	'Curvature',[1, 1], 'FaceColor', c, 'EdgeColor', c);

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