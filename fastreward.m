function [val,grad] = fastreward(u, s, mdp_data, mdp, reward)

% Reshape.
u = reshape(u, size(u, 1)/mdp_data.udims,mdp_data.udims);

% Compute reward and its gradient.
[val, drdu] = evalreward(reward, mdp_data, s, u);

% Compute gradient of reward with respect to control.
grad = -drdu(:);
val = -sum(val);


function [r, g] = evalreward(reward, mdp_data, s, u);

% Compute states and state Hessian.
[states, A, B] = crowdworldcontrol(mdp_data, s, u);

theta = reward.theta;

% Get constants.
T = size(u, 1);
Du = size(u, 2);
Dx = size(states, 2);

r = zeros(T, 1); % all features write here
drdx = zeros(T, Dx); % x-features write here
g = zeros(T, Du); % u-features write here

% Collect contributions by features
for f = 1:length(theta)
	switch reward.features{f}.type
	case 'acc2sum'
		% u-feature
		[cr, cg] = acc2sumevalreward(reward.features{f}, mdp_data, s, u);
		%disp(sum(cr))
		%disp(size(cg))
		%disp(length(reward.features{f}.idx))
		%disp(size(g, 1))

		%disp(sum(sum(cg)))

		g = g + theta(f)*cg;
	otherwise 
		% x-feature
		[cr, cdrdx] = evaluatexfeature(reward.features{f}, mdp_data, states);
		%disp(sum(cr))
		%disp(reward.features{f}.type)
		drdx = drdx + theta(f)*cdrdx;
	
		%check_drdx = zeros(T, Dx);
		%check_drdx(:, reward.features{f}.idx) = cdrdx;
		%disp(sum(sum(permute(gradprod(A,B,permute(check_drdx,[1 3 2])),[1 3 2]))))
	end
	r = r + theta(f)*cr;
end

% Combine contributions by x- and u-features
g = g + permute(gradprod(A, B, permute(drdx, [1 3 2])), [1 3 2]);

function [r, drdx] = evaluatexfeature(reward, mdp_data, states)
switch reward.type
case 'verr2sum'
	[r, ~, ~, ~, drdx] = verr2sumevalreward(reward, mdp_data, 0, 0, states);
case 'dgaussiansum'
	[r, ~, ~, ~, drdx] = dgaussiansumevalreward(reward, mdp_data, 0, 0, states);
case 'iesum'
	[r, ~, ~, ~, drdx] = iesumevalreward(reward, mdp_data, 0, 0, states);
end