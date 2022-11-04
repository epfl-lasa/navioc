function X_est = kalmanfilter(Pos, T, T_est, init_vel) %, Q, R, sigma)
%KALMANFILTER Summary of this function goes here
%   Detailed explanation goes here

Q = eye(6);
R = eye(2);
sigma = 50;


H = [eye(2), zeros(2, 4)];

n_obs = length(T);

X = zeros(n_obs, 6);

x_pred = zeros(6, 1);
P_prior = sigma*eye(6);

x_pred(1:2) = Pos(1, :);
if nargin == 4
    x_pred(3:4) = init_vel;
end

for k = 1:n_obs
    % correct state at k
    K = P_prior*H'/(H*P_prior*H' + R);
    
    z = Pos(k, :)';
    
    x = x_pred + K*(z - H*x_pred);
    
    P = (eye(6) - K*H)*P_prior;
    
    % predict state at k + 1
    if k ~= n_obs
        dt = T(k + 1) - T(k);
        A = [eye(2), eye(2)*dt, eye(2)*dt^2/2; ...
            zeros(2), eye(2), eye(2)*dt; ...
            zeros(2), zeros(2), eye(2)];
        
        x_pred = A*x;
        P_prior = A*P*A' + Q;
    end
    
    X(k, :) = x';
end

n_est = length(T_est);

X_est = ones(n_est, 6)*nan;

k = 0;
for k_est = 1:n_est
    t_est = T_est(k_est);
    
    while true
        if k == n_obs || T(k + 1) > t_est
            break
        end
        k = k + 1;
    end
    
    if k == 0
        continue
    end
        
    dt = t_est - T(k);   
    A = [eye(2), eye(2)*dt, eye(2)*dt^2/2; ...
        zeros(2), eye(2), eye(2)*dt; ...
        zeros(2), zeros(2), eye(2)];
    X_est(k_est, :) = (A*X(k, :)')';
end

end

