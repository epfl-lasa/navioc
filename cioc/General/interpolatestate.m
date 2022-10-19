% Interpolate states in a discretized MDP.
function [sa_s,sa_p] = interpolatestate(next_state,discrete_mdp,mdp_data,action)

if nargin < 4,
    action = 0;
end;

% Choose if we're interpolating actions or states.
if action,
    cells = discrete_mdp.cells_action;
    if action > 1,
        bounds = sign(discrete_mdp.abounds).*sqrt(abs(discrete_mdp.abounds));
        next_state = sign(next_state).*sqrt(abs(next_state));
    else
    	bounds = discrete_mdp.abounds;
    	next_state = next_state;
    end;
    dims = mdp_data.udims;
else
    cells = discrete_mdp.cells_state;
    bounds = discrete_mdp.sbounds;
    dims = mdp_data.dims;
end;

step_size = (bounds(2,:)-bounds(1,:))/cells;
next_state = bsxfun(@minus,next_state,bounds(1,:)); % Subtract lower bounds.
lidx = min(cells+1,max(1,floor(bsxfun(@rdivide,next_state,step_size))+1));
uidx = min(cells+1,max(1,ceil(bsxfun(@rdivide,next_state,step_size))+1));
frac = min(1.0,max(0,(bsxfun(@rdivide,next_state,step_size)+1-lidx)));
dim_coord(1,:,1) = lidx;
dim_coord(1,:,2) = uidx;
dim_fract(1,:,1) = 1-frac;
dim_fract(1,:,2) = frac;
sa_s = zeros(1,1,2^dims);
sa_p = zeros(1,1,2^dims);
for k=1:2^dims,
    nncoord = ind2coord(k,2*ones(dims,1));
    nextcoord = zeros(1,dims);
    nextprob = 1;
    for m=1:dims,
        nextprob = nextprob.*dim_fract(:,m,nncoord(m));
        nextcoord(:,m) = dim_coord(:,m,nncoord(m));
    end;
    sa_s(1,1,k) = coord2ind(nextcoord,(cells+1)*ones(1,dims));
    sa_p(1,1,k) = nextprob;
end;
