% Evaluate the distance a car and its partial derivatives.
function [r1,r2,dr1dx,dr2dx] = highwaycareval(states,reward)

% Get points.
pts = states(:,1:2);

% Compute car points at corresponding times.
difft = abs(bsxfun(@minus,states(:,6),(reward.x(:,6))'));
[~,idx] = sort(difft,2);
idx1 = (reward.x(idx(:,1),6) < reward.x(idx(:,2),6)).*idx(:,1) + (reward.x(idx(:,2),6) <= reward.x(idx(:,1),6)).*idx(:,2);
idx2 = (reward.x(idx(:,1),6) >= reward.x(idx(:,2),6)).*idx(:,1) + (reward.x(idx(:,2),6) > reward.x(idx(:,1),6)).*idx(:,2);
fr = states(:,6)-reward.x(idx1,6);
cp = bsxfun(@times,reward.x(idx1,1:2),1-fr) + bsxfun(@times,reward.x(idx2,1:2),fr);
theta = bsxfun(@times,reward.x(idx1,3),1-fr) + bsxfun(@times,reward.x(idx2,3),fr);
cosine = cos(theta);
sine = sin(theta);

% Compute distance.
d1 = pts(:,1)-cp(:,1);
d2 = pts(:,2)-cp(:,2);
r1 = d1.*cosine - d2.*sine;
r2 = d1.*sine + d2.*cosine;

if nargout >= 3,
    % Compute r1 and r2 gradients.
    dr1dx = [cosine, -sine];
    dr2dx = [sine, cosine];
end;

% The Hessians are simply zero.
