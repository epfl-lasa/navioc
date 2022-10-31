function fit_batch = addvdestobatch(fit_batch, y_1, y_2, dx_min, v_min)%i, j)
for i = 1:length(fit_batch.fits)
	if ~fit_batch.fits{i}.empty
		Vmag = sqrt(sum(fit_batch.fits{i}.fit.V_opt.^2, 2));
		Phi_ref = estphiref(fit_batch.fits{i}.fit,  y_1, y_2, dx_min);
		speed_ref = estspeedref(Vmag, v_min, 3, 16);
		fit_batch.fits{i}.V_ref = [cos(Phi_ref), sin(Phi_ref)]*speed_ref;
		fit_batch.fits{i}.V_ref(Vmag < v_min, :) = 0.0;
	end
end