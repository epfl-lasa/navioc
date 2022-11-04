function showfitbatch(fit_batch)

subplot(1, 2, 1)
hold on
for i = 1:length(fit_batch.fits)
	if ~fit_batch.fits{i}.empty %&& fit_batch.fits{i}.type == "ped"
		plot(fit_batch.fits{i}.X(:, 1), fit_batch.fits{i}.X(:, 2), "k")
	elseif ~fit_batch.fits{i}.empty
		plot(fit_batch.fits{i}.X(:, 1), fit_batch.fits{i}.X(:, 2), "g")
	end
end
% for i = 1:length(fit_batch.fits)
% 	if ~fit_batch.fits{i}.empty && (...
% 		fit_batch.fits{i}.type == "powered_wheelchair" || ...
% 		fit_batch.fits{i}.type == "manual_wheelchair")
% 		plot(fit_batch.fits{i}.X(:, 1), fit_batch.fits{i}.X(:, 2), "r")
% 	end
% end
daspect([1, 1, 1])
subplot(1, 2, 2)
hold on
for i = 1:length(fit_batch.fits)
	if ~fit_batch.fits{i}.empty% && fit_batch.fits{i}.type == "ped"
		plot(fit_batch.fits{i}.fit.X_opt(:, 1), fit_batch.fits{i}.fit.X_opt(:, 2), "k")
	elseif ~fit_batch.fits{i}.empty
		plot(fit_batch.fits{i}.fit.X_opt(:, 1), fit_batch.fits{i}.fit.X_opt(:, 2), "g")
	end
end
% for i = 1:length(fit_batch.fits)
% 	if ~fit_batch.fits{i}.empty && (...
% 		fit_batch.fits{i}.type == "powered_wheelchair" || ...
% 		fit_batch.fits{i}.type == "manual_wheelchair")
% 		plot(fit_batch.fits{i}.fit.X_opt(:, 1), fit_batch.fits{i}.fit.X_opt(:, 2), "r")
% 	end
% end
daspect([1, 1, 1])

% for i = 1:length(fit_batch.fits)
% 	if ~fit_batch.fits{i}.empty
% 		ind = tracks{i}.t(1) <= interpolations{i}.T & interpolations{i}.T <= tracks{i}.t(end);
% 		plot(fit_batch.fits{i}.X(ind, 1), fit_batch.fits{i}.X(:, 2), ...
% 			"k--", 'LineWidth', 2)
% 	end
% end
% for i = 1:length(tracks)
%     if ~isempty(interpolations{i})
%         ind = tracks{i}.t(1) <= interpolations{i}.T & interpolations{i}.T <= tracks{i}.t(end);
%         plot(interpolations{i}.P(ind, 1), interpolations{i}.P(ind, 2), "r")
%     end
% end
return

% figure(2)
% subplot(2, 1, 1)
% hold on
% for i = 1:length(tracks)
% 	plot(tracks{i}.x, tracks{i}.y)
% end
% daspect([1, 1, 1])
% subplot(2, 1, 2)
% hold on
% for i = 1:length(tracks)
%     if ~isempty(interpolations{i})
%         ind = tracks{i}.t(1) <= interpolations{i}.T & interpolations{i}.T <= tracks{i}.t(end);
%         plot(interpolations{i}.P(ind, 1), interpolations{i}.P(ind, 2))
%     else
%         plot(nan,nan)
%     end
% end
% daspect([1, 1, 1])