function [states, A, B, invB, dxdu, d2xdudu] = crowdworldcontrol(mdp_data, x, u)

Dx = mdp_data.dims;
Du = mdp_data.udims;
T = size(u, 1);
h = mdp_data.time_step;

At = [...
	eye(Du), 	eye(Du)*h; 	...
	zeros(Du), 	eye(Du) ...
];
Bt = [...
	eye(Du)*h^2/2; ...
	eye(Du)*h ...
];

states = zeros(T, Dx);
states(1, :) = (At*x' + Bt*u(1, :)')';
for t = 2:T
	states(t, :) = (At*states(t - 1, :)' + Bt*u(t, :)')';
end

if nargout >= 2
    A = repmat(At, [1, 1, T]);
    B = repmat(Bt, [1, 1, T]);
    invBt = pinv(Bt);
    invB = repmat(invBt, [1, 1, T]);
end

if nargout >= 5
	dxdu = zeros(Du*T, Dx*T);
	product = Bt;
	for j = 1:T
		for d = 0:(T - j)
			id = 1 + d;
			jd = j + d;
			dxdu((1 + (id - 1)*Du):(id*Du), (1 + (jd - 1)*Du*2):(jd*Du*2)) = product';
		end
		product = At*product;
	end
end

if nargout >= 6
	d2xdudu = zeros(1,1,1);% zeros(Du*T,Du*T,Dx*T);
end