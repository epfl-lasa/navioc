% Compute the OptV objective.
function [val,grad] = optvcost(w,pallxpr,xprf,allxprf)

% Inputs:
% pxpr p(x'|x), N x 1 array
% pallxpr p(x'_a|x), N x A array
% xprf f(x'), N x 1 x F array
% allxprf f(x'_a), N x A x F array

% Permute weights.
w3 = permute(w,[2 3 1]);

% Compute likelihood.
asums = -sum(bsxfun(@times,allxprf,w3),3);
maxx = max(asums,[],2);
asums = bsxfun(@minus,asums,maxx);
G = sum(bsxfun(@times,pallxpr,exp(asums)),2);
val = sum(sum(bsxfun(@times,xprf,w3),3) + log(G) + maxx,1);

% Compute gradients.
grad = sum(xprf + bsxfun(@rdivide,sum(bsxfun(@times,pallxpr,bsxfun(@times,exp(asums),-allxprf)),2),G),1);
grad = permute(grad,[3 2 1]);

% Regularize.
%prior = 0.1;
%grad = grad + prior*2.0*w;
%val = val + prior*(w'*w);
