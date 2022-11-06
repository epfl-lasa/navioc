function F = fstats(features, samples_data, plot_hist)

if nargin < 3
	plot_hist = false;
end

orig_state = warning;
warning('off','all');
rmpath Features
rmpath Crowdworld
warning(orig_state);

addpath Features2
addpath Crowdworld2
addpath cioc/General
addpath cioc/Reward
addpath cioc/FastHess
addpath cioc/Utilities/minFunc
addpath cioc/Auglag
addpath cioc/Laplace

infos = trajinfos(features, 'crowdworld', ...
	samples_data.mdp_data_arr, samples_data.samples);

F = zeros(96, length(features), length(infos));
for k = 1:length(infos)
	F(:, :, k) = infos{k}.f;
end

if plot_hist
	for j = 1:length(features)
		figure
		hold on
		histogram(flat(F(:, j, :)), 'HandleVisibility', 'off')
		xline(mean(flat(F(:, j, :))), "k", 'Mean')
		xline(median(flat(F(:, j, :))), "k--", 'Median')
		title(features{j}.type)
		legend()
	end
end


function xline(x, linespec, label)
plot([x, x], getfield(gca(), 'YLim'), linespec, 'DisplayName', label)

function y = flat(X)
y = reshape(X, [numel(X), 1]);