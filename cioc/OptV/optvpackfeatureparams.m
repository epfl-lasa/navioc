% Pack feature means and covariances into theta vector.
function theta = optvpackfeatureparams(means,covars)

% Note that covars is actually precision matrices.

% Constants.
F = size(covars,3);
D = size(covars,1);

% Compute theta entries per feature.
TPF = 1 + D + 0.5*D*(D+1);

% Allocate theta.
theta = zeros(F*TPF,1);

% Step over each feature.
for f=1:F,
    strt = (f-1)*TPF;
    theta(strt+1) = -0.5*means(f,:)*covars(:,:,f)*means(f,:)';
    theta((strt+1)+(1:D)) = covars(:,:,f)*means(f,:)';
    idx = 0;
    for d=1:D,
        theta((strt+1+D+idx)+(1:d)) = -covars(1:d,d,f);
        theta((strt+1+D+idx)+d) = theta((strt+1+D+idx)+d)*0.5;
        idx = idx + d;
    end;
end;
