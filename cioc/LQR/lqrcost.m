% Evaluate the approximate MaxEnt objective (LQR variant).
function [val,grad] = lqrcost(theta,infos)

val = 0;
grad = zeros(1,size(theta,1));

for i=1:length(infos),
    % Constants.
    T = size(infos{i}.f,1);
    F = size(infos{i}.f,2);
    Du = size(infos{i}.Ht,3);
    Dx = size(infos{i}.Hh,3);
    
    % Initialize Vhat and vhat.
    Vhat = zeros(Dx,Dx);
    vhat = zeros(Dx,1);
    Hess = zeros(Du,Du);
    Hessinv = zeros(Du,Du);
    Grad = zeros(Du,1);
    Vhatgrad = zeros(Dx,Dx,F);
    vhatgrad = zeros(Dx,1,F);
    Hessgrad = zeros(Du,Du,F);
    Gradgrad = zeros(Du,1,F);
    
    % Precompute quantities.
    Hhgrad = permute(infos{i}.Hh,[3 4 1 2]);
    Htgrad = permute(infos{i}.Ht,[3 4 1 2]);
    ghgrad = permute(infos{i}.gh,[3 4 1 2]);
    gtgrad = permute(infos{i}.gt,[3 4 1 2]);
    A = infos{i}.A;
    B = infos{i}.B;
    Hh = permute(...
        sum(bsxfun(@times,infos{i}.Hh,theta'),2),...
        [3 4 1 2]);
    Ht = permute(...
        sum(bsxfun(@times,infos{i}.Ht,theta'),2),...
        [3 4 1 2]);
    gh = permute(...
        sum(bsxfun(@times,infos{i}.gh,theta'),2),...
        [3 4 1 2]);
    gt = permute(...
        sum(bsxfun(@times,infos{i}.gt,theta'),2),...
        [3 4 1 2]);
    
    % Go from start to end.
    for t=T:-1:1,        
        % Update Vhat, vhat, Vhatgrad, and vhatgrad.
        Vhatold = Vhat;
        vhatold = vhat;
        Hessold = Hess;
        Hessinvold = Hessinv;
        Gradold = Grad;
        if t == T,
            vhat = gh(:,:,t);
            Vhat = Hh(:,:,t);
        else
            vhat = gh(:,:,t) + A(:,:,t+1)'*vhat - ...
                A(:,:,t+1)'*Vhat*B(:,:,t+1)*Hessinv*(gt(:,:,t+1) + B(:,:,t+1)'*vhat);
            Vhat = Hh(:,:,t) + ...
                A(:,:,t+1)'*Vhat*A(:,:,t+1) - ...
                A(:,:,t+1)'*Vhat'*B(:,:,t+1)*Hessinv*(B(:,:,t+1)'*Vhat*A(:,:,t+1));
        end;
        
        % Compute Hess and Grad.
        Hess = Ht(:,:,t) + B(:,:,t)'*Vhat*B(:,:,t);
        Grad = gt(:,:,t) + B(:,:,t)'*vhat;
        Hessinv = inv(Hess);
        
        % Compute likelihood.
        Hdet = det(-Hess);
        if Hdet < 0.0,
            val = val - 1e100;
        end;
        val = val + 0.5*Grad'*Hessinv*Grad + 0.5*log(Hdet);
        
        for f=1:F,
            % Compute Vhat and vhat gradients.
            if t == T,
                vhatgrad(:,:,f) = ghgrad(:,:,t,f);
                Vhatgrad(:,:,f) = Hhgrad(:,:,t,f);
            else
                vhatgrad(:,:,f) = ghgrad(:,:,t,f) + A(:,:,t+1)'*vhatgrad(:,:,f) - ...
                    A(:,:,t+1)'*Vhatold*B(:,:,t+1)*Hessinvold*(gtgrad(:,:,t+1,f) + B(:,:,t+1)'*vhatgrad(:,:,f)) - ...
                    A(:,:,t+1)'*Vhatgrad(:,:,f)*B(:,:,t+1)*Hessinvold*(gt(:,:,t+1) + B(:,:,t+1)'*vhatold) + ...
                    A(:,:,t+1)'*Vhatold*B(:,:,t+1)*Hessinvold*Hessgrad(:,:,f)*Hessinvold*(gt(:,:,t+1) + B(:,:,t+1)'*vhatold);
                Vhatgrad(:,:,f) = Hhgrad(:,:,t,f) + A(:,:,t+1)'*Vhatgrad(:,:,f)*A(:,:,t+1) - ...
                    A(:,:,t+1)'*Vhatold'*B(:,:,t+1)*Hessinvold*B(:,:,t+1)'*Vhatgrad(:,:,f)*A(:,:,t+1) - ...
                    A(:,:,t+1)'*Vhatgrad(:,:,f)'*B(:,:,t+1)*Hessinvold*B(:,:,t+1)'*Vhatold*A(:,:,t+1) + ...
                    A(:,:,t+1)'*Vhatold'*B(:,:,t+1)*Hessinvold*Hessgrad(:,:,f)*Hessinvold*B(:,:,t+1)'*Vhatold*A(:,:,t+1);
            end;

            % Compute Hess and Grad gradients.
            Hessgrad(:,:,f) = Htgrad(:,:,t,f) + B(:,:,t)'*Vhatgrad(:,:,f)*B(:,:,t);
            Gradgrad(:,:,f) = gtgrad(:,:,t,f) + B(:,:,t)'*vhatgrad(:,:,f);
        
            % Add to gradients.
            grad(f) = grad(f) + Gradgrad(:,:,f)'*Hessinv*Grad - ...
                0.5*Grad'*Hessinv*Hessgrad(:,:,f)*Hessinv*Grad + ...
                0.5*sum(sum(Hessinv.*Hessgrad(:,:,f)));
        end;
    end;
end;

% Negate objective and gradient.
val = -val;
grad = -grad';
