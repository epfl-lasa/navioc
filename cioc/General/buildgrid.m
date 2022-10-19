% Build grid for discretization.
function vals = buildgrid(bounds,cells,quad)

if quad,
    bounds = sign(bounds).*sqrt(abs(bounds));
end;

D = size(bounds,2);
SPLITS = cells+1;
step_size = (bounds(2,:)-bounds(1,:))/cells;

% Create cell array of positions.
if D == 1,
    vcell = cell(SPLITS,1);
else
    vcell = cell(SPLITS*ones(1,D));
end;
all_inds = 1:SPLITS^D;
all_subs = zeros(SPLITS^D,D);
for k=1:D,
    sub_mat = repmat(1:SPLITS,horzcat(SPLITS*ones(1,k-1),1,SPLITS*ones(1,D-k)));
    all_subs(:,k) = sub_mat(:);
    vk = bounds(1,k)+(0:(SPLITS-1))*step_size(k);
    for i=all_inds,
        vcell{i}(k) = vk(all_subs(i,k));
    end;
end;

% Convert to array.
vals = cell2mat(vcell(:));

if quad,
    vals = sign(vals).*(vals.^2);
end;
