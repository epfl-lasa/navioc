% Compute input vector feature function S(x)
function s = optvs(x,grad)

% x is N x A x D
% s is N x A x 1 x S
% if grad is on, s is N x A x 1 x S x D

if nargin == 1,
    grad = 0;
end;

% Constants.
N = size(x,1);
A = size(x,2);
D = size(x,3);

% Compute theta entries per feature.
TPF = 1 + D + 0.5*D*(D+1);

% Allocate.
if grad,
    s = zeros(N,A,1,TPF,D);

    % Write constant entries.
    s(:,:,1,1+(1:D),:) = repmat(permute(eye(D),[3 4 5 1 2]),[N A 1 1 1]);

    % Write quadratic entries.
    idx = 0;
    for d1=1:D,
        for d2=1:d1,
            s(:,:,1,(1+D+idx)+d2,d2) = x(:,:,d1);
            s(:,:,1,(1+D+idx)+d2,d1) = s(:,:,1,(1+D+idx)+d2,d1) + x(:,:,d2);
        end;
        idx = idx + d1;
    end;
else
    s = ones(N,A,1,TPF);

    % Write constant entries.
    s(:,:,1,1+(1:D)) = permute(x,[1 2 4 3]);

    % Write quadratic entries.
    idx = 0;
    for d=1:D,
        s(:,:,1,(1+D+idx)+(1:d)) = permute(bsxfun(@times,x(:,:,d),x(:,:,1:d)),[1 2 4 3]);
        idx = idx + d;
    end;
end;
