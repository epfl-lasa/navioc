function play(tracks, tracks_fits, bins, fig_limits, t_1, t_2, walls)

steppers = ones(1, length(tracks));
steppers_fit = ones(1, length(tracks));

dt_bin = bins.dt_bin;

tMin = bins.t_min;
t_1 = tMin + t_1;
t_2 = tMin + t_2;
fps = 10;
T = t_1:(1/fps):t_2;
tClock = clock;
fig = figure(1);
set_up_key_press_fcn(fig);

%for j = 1:length(T)
j = 1;
while true

	t = T(j);
	i = 1 + floor((t - tMin)/dt_bin);

	XY = ones(length(bins.registries{i}), 2)*nan;
	XY_fit = XY;
	%Vdes = XY;
	is_wheelchair = logical(zeros(length(bins.registries{i}), 1));
	is_stroller = logical(zeros(length(bins.registries{i}), 1));
	for l = 1:length(bins.registries{i})
		k = bins.registries{i}(l);

		if (tracks{k}.type == "manual_wheelchair" || ...
			tracks{k}.type == "powered_wheelchair")
			is_wheelchair(l) = true;
		elseif tracks{k}.type == "stroller"
			is_stroller(l) = true;
		end
		

		while steppers(k) + 1 <= length(tracks{k}.t) && tracks{k}.t(steppers(k) + 1) < t
			steppers(k) = steppers(k) + 1;
		end

		if steppers(k) + 1 <= length(tracks{k}.t) && tracks{k}.t(steppers(k)) < t
			m = steppers(k);
			XY(l, :) = interpolate(t, ...
				tracks{k}.t(m), [tracks{k}.x(m), tracks{k}.y(m)], ...
				tracks{k}.t(m+1), [tracks{k}.x(m+1), tracks{k}.y(m+1)]);
		end
		if ~tracks_fits{k}.empty
			while steppers_fit(k) + 1 <= length(tracks_fits{k}.fit.T_opt) && tracks_fits{k}.fit.T_opt(steppers_fit(k) + 1) < t
				steppers_fit(k) = steppers_fit(k) + 1;
			end

			if steppers_fit(k) + 1 <= length(tracks_fits{k}.fit.T_opt) && tracks_fits{k}.fit.T_opt(steppers_fit(k)) < t
				m = steppers_fit(k);
				XY_fit(l, :) = interpolate(t, ...
					tracks_fits{k}.fit.T_opt(m), [tracks_fits{k}.fit.X_opt(m, 1), tracks_fits{k}.fit.X_opt(m, 2)], ...
					tracks_fits{k}.fit.T_opt(m+1), [tracks_fits{k}.fit.X_opt(m+1, 1), tracks_fits{k}.fit.X_opt(m+1, 2)]);
				%Vdes(l, :) = interpolate(t, ...
				%	tracks_fits{k}.fit.T_opt(m), [tracks_fits{k}.v_des(m, 1), tracks_fits{k}.v_des(m, 2)], ...
				%	tracks_fits{k}.fit.T_opt(m+1), [tracks_fits{k}.v_des(m+1, 1), tracks_fits{k}.v_des(m+1, 2)]);
			end
		end
	end

	hold on

	for w = 1:size(walls, 1)
		plot(walls(w, 1:2), walls(w, 3:4), "k")
	end

	plot(XY(:, 1), XY(:, 2), "k.")
	plot(XY_fit(:, 1), XY_fit(:, 2), "r.")

	plot(XY(is_wheelchair, 1), XY(is_wheelchair, 2), "kx")
	plot(XY_fit(is_wheelchair, 1), XY_fit(is_wheelchair, 2), "rx")

	plot(XY(is_stroller, 1), XY(is_stroller, 2), "ks")
	plot(XY_fit(is_stroller, 1), XY_fit(is_stroller, 2), "rs")

	%quiver(XY_fit(:, 1), XY_fit(:, 2), Vdes(:, 1), Vdes(:, 2))

	text(XY(:, 1), XY(:, 2), string(bins.registries{i}))
	hold off

    daspect([1,1,1])
    xlim(fig_limits(1:2))
    ylim(fig_limits(3:4))

    minutes = floor(t/60.0);
    seconds = floor(t - minutes*60.0);
    title(sprintf("t = %i:%i = %.1f, t1 = %.1f s", minutes, seconds, t, t-tMin))

    pause(max([0.001, 1/fps - etime(clock, tClock)]))
    if rewind(fig)
    	j = max([0, j - fps]);
    	steppers = ones(1, length(tracks));
    	steppers_fit = ones(1, length(tracks));
    end
    if ~ishandle(fig)
    	break
    end
      
	j = j + 1;
	if j == length(T)
		break
	end

    tClock = clock;
    clf
end


function xy = interpolate(t, t1, xy1, t2, xy2)
xy = (xy1*(t2 - t) + xy2*(t - t1))/(t2 - t1);

function set_up_key_press_fcn(fig)
hManager = uigetmodemanager(fig);
try
    set(hManager.WindowListenerHandles, 'Enable', 'off');  % HG1
catch
    [hManager.WindowListenerHandles.Enabled] = deal(false);  % HG2
end
set(fig, 'WindowKeyPressFcn', []);
set(fig, 'KeyPressFcn', @toggle_play_pause);
global play_pause
play_pause = false;

function toggle_play_pause(fig, key_data)
global play_pause
global command_rewind
play_pause = ~play_pause;
if strcmp(key_data.Key, 'leftarrow')
	command_rewind = true;
end

function flag = rewind(fig)
global command_rewind
global play_pause
if command_rewind
	flag = true;
	command_rewind = false;
else
	while play_pause && ishandle(fig)
	    pause(0.01)
	end
	if command_rewind
		flag = true;
		command_rewind = false;
		play_pause = true;
	else
		flag = false;
	end
end