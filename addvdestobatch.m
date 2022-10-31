function fit_batch = addvdestobatch(fit_batch, y_1, y_2, dx_min)%i, j)

% if nargin == 2
% 	s1 = sprintf('s_p00001/fit_batches/fit_batch_%i_%i.mat', i, j);
% 	s2 = sprintf('s_p00001/fit_batches_vdes/fit_batch_%i_%i.mat', i, j);
% else
% 	s1 = sprintf('fit_batches/batch_%i.mat', i);
% 	s2 = sprintf('fit_batches_vdes/batch_%i.mat', i);
% end

% fit_batch = vario(s1, 'fit_batch');

for i = 1:length(fit_batch.fits)
	if ~fit_batch.fits{i}.empty
		Vmag = sqrt(sum(fit_batch.fits{i}.fit.V_opt.^2, 2));
		vmag_mode = findmode(Vmag, [0.0, 2], 11);
		VxAbs = abs(fit_batch.fits{i}.fit.V_opt(:, 1));
		vxabs_mode = findmode(VxAbs, [0.0, 2], 11);
		fit_batch.fits{i}.vmag_des = vmag_mode;
		fit_batch.fits{i}.vxabs_des = vxabs_mode;
		Phi_ref = estimorientref(fit_batch.fits{i}.fit.X_opt, y_1, y_2, dx_min);
		speed_ref = estimspeedref(Vmag, 0.3, 3, 16);
		fit_batch.fits{i}.V_ref = [cos(Phi_ref), sin(Phi_ref)]*speed_ref;
	end
end

%vario(s2, 'fit_batch', fit_batch);