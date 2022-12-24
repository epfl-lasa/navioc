function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    acc2sumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

normalizer = mdp_data.n_ped*reward.expec;

Nt = size(u, 1);
Nu = mdp_data.udims;
Nx = mdp_data.dims;

if reward.type_other
    idx = mdp_data.idx_other;
elseif reward.type_w
    idx = mdp_data.idx_wheelchair;
elseif reward.type_c
    idx = mdp_data.idx_companion;
else
    idx = 1:(Nu/2);
end

n = 2*length(idx);
jj = reshape([idx*2 - 1; idx*2], [1, n]);

r = 0.5*sum(u(:, jj).^2, 2)/normalizer;
if nargout >= 2
    g = zeros(Nt, Nu);
    g(:, jj) = u(:, jj)/normalizer;
    %g = u;
end

if nargout >= 3
    drdu = g;
    d2rdudu = zeros(Nt, Nu, Nu);
    d2rdudu(:, jj, jj) = repmat(reshape(eye(n), [1, n, n]), [Nt, 1, 1])/normalizer;
    %repmat(reshape(eye(Nu), [1, Nu, Nu]), [Nt, 1, 1]);
end

if nargout >= 5
    drdx = zeros(Nt, Nx);
    d2rdxdx = zeros(Nt, Nx, Nx);
    %disp(reward.type)
end