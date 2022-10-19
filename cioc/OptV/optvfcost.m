% Compute the OptV objective for feature optimization.
function [val,grad] = optvfcost(theta,pallxpr,allxpr,xpr,F)

% Inputs:
% xpr, N x D array
% allxpr, N x A x D array

global w;

% Constants.
D = size(allxpr,3);

% Compute theta entries per feature.
TPF = 1 + D + 0.5*D*(D+1);

% Permute theta.
theta3 = permute(reshape(theta,TPF,F),[3 4 2 1]);
theta3 = min(max(theta3,-1.0e50),1.0e50);

% Precompute f(xpr) for xpr.
xprf = exp(sum(bsxfun(@times,optvs(permute(xpr,[1 3 2])),theta3),4));
xprf = min(xprf,1.0e100);
xprf = bsxfun(@rdivide,xprf,max(sum(xprf,3),1.0e-20));

% Precompute f(allxpr) for allxpr.
allxprf = exp(sum(bsxfun(@times,optvs(allxpr),theta3),4));
allxprf = min(allxprf,1.0e100);
allxprf = bsxfun(@rdivide,allxprf,max(sum(allxprf,3),1.0e-20));

% Set up optimization options.
options = struct();
options.Method = 'lbfgs';
options.Display = 'none';
%options.DerivativeCheck = 'on';
options.maxIter = 1500;
options.MaxFunEvals = 1500;

% Optimize weights.
w = minFunc(@(w)optvcost(w,pallxpr,xprf,allxprf),w,options);
    
% Flip weights.
w3 = permute(w,[2 3 1]);

% Compute likelihood.
asums = -sum(bsxfun(@times,allxprf,w3),3);
maxx = max(asums,[],2);
asums = bsxfun(@minus,asums,maxx);
G = sum(bsxfun(@times,pallxpr,exp(asums)),2);
val = sum(sum(bsxfun(@times,xprf,w3),3) + log(G) + maxx,1);

% Compute gradients.
grad = sum(bsxfun(@times,bsxfun(@times,xprf,bsxfun(@minus,w3,sum(bsxfun(@times,xprf,w3),3))),optvs(permute(xpr,[1 3 2]))) - ...
    bsxfun(@rdivide,sum(bsxfun(@times,pallxpr,bsxfun(@times,exp(asums),...
    bsxfun(@times,bsxfun(@times,allxprf,bsxfun(@minus,w3,sum(bsxfun(@times,allxprf,w3),3))),optvs(allxpr)))),2),G),1);

% Compute gradients.
grad = permute(grad,[4 3 2 1]);
grad = grad(:);

% Regularize.
%prior = 0.1;
%grad = grad + prior*2.0*theta;
%val = val + prior*(theta'*theta);
