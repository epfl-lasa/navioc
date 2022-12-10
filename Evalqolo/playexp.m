function playexp(mat, dt_start, n_skip, n_skip_rob, kill, dt_memory, show_plan, videoname)
if nargin < 8
    videoname = 'experiment.avi';
end
if nargin < 7
    show_plan = true;
end
if nargin < 6
    dt_memory = 5.0;
end
if nargin < 5
	kill = true;
end
if nargin < 4
	n_skip_rob = 50;
end
if nargin < 3
	n_skip = 5;
end
if nargin < 2
	dt_start = 0.0;
end

h = 0.05;

fig = figure('Position', [300, 600, 2000, 600]);
ax = axes(fig);

set_up_key_press_fcn(fig);

t = (mat.odom_sample.t(1) + dt_start):0.05:mat.odom_sample.t(end);
nt = length(t);
frames = cell(1, nt);
n_ped = size(mat.tracked_persons.isdef, 2);

if n_ped > 7
	C = repmat(colororder, [ceil(n_ped/7), 1]);
else
	C = colororder;
end

x_limits = [-1, 17];
y_limits = [-3, 3];

tracks_def = mat.tracked_persons.isdef;
tracks_t = mat.tracked_persons.t;
if mat.tracked_persons.n_msg ~= 0
	tracks_P = mat.tracked_persons.states(:, :, 1:2);
	tracks_V = mat.tracked_persons.states(:, :, 3:4);
else
	tracks_P = ones(0, 0, 2);
	tracks_V = ones(0, 0, 2);
end	
tmp_handles = [];
i = 0;
while i < nt
	i = i + 1;
	delete(tmp_handles)
	tmp_handles = [];
	
	hold(ax, 'on')

	% TRACKED_PERSONS
	ii_ = find(tracks_t <= t(i) & tracks_t >= (t(i) - dt_memory));
	for i_ = ii_
		if i_ ~= ii_(end) && mod(i_, n_skip) ~= 0
			continue
		end
		w1 = (t(i) - tracks_t(i_))/dt_memory;
		w0 = 1 - w1;
		for j = 1:n_ped
			if tracks_def(i_, j) && (~kill || tracks_def(ii_(end), j))
				tmp_handles = [tmp_handles, ...
					plotcircle(ax, tracks_P(i_, j, 1), tracks_P(i_, j, 2), 0.2, C(j, :)*w0 + w1)];
			end
		end
	end

	% ODOM_SAMPLE
	ii_odom = find(mat.odom_sample.t <= t(i) & mat.odom_sample.t >= (t(i) - dt_memory));
	for i_ = ii_odom
		if i_ ~= ii_odom(end) && mod(i_, n_skip_rob) ~= 0
			continue
		end
		w1 = (t(i) - mat.odom_sample.t(i_))/dt_memory;
		w0 = 1 - w1;
		tmp_handles = [tmp_handles, ...
			plotcircle(ax, mat.odom_sample.states(i_, 1), mat.odom_sample.states(i_, 2), 0.2, w1*ones(1,3))];
	end

	% OC_REPLY
	ii_oc = find(mat.oc_reply.t <= t(i) & mat.oc_reply.t >= (t(i) - dt_memory));
	if ~isempty(ii_oc) && show_plan
		i_1 = ii_oc(1);
		i_2 = ii_oc(end);
		for i_m = i_1:(i_2-1)
			w1 = (t(i) - mat.oc_reply.t(i_m))/dt_memory;
			w0 = 1 - w1;
			C_m = w0*[0, 1, 0] + w1*[1, 1, 1];
			h_oc_1 = plot(mat.oc_reply.states(i_m, :, 1)', mat.oc_reply.states(i_m, :, 2)', 'Color', C_m, 'LineStyle', ':')';
			tmp_handles = [tmp_handles, h_oc_1];
		end
		h_oc_2 = plot(mat.oc_reply.states(i_2, :, 1), mat.oc_reply.states(i_2, :, 2), 'g');
		tmp_handles = [tmp_handles, h_oc_2];
	end

	% VELOCITIES
	XY = zeros(0, 2);
	UV = zeros(0, 2);
	if ~isempty(ii_)
		UV = [UV; reshape(tracks_V(ii_(end), tracks_def(ii_(end), :), :), [], 2)];
		XY = [XY; reshape(tracks_P(ii_(end), tracks_def(ii_(end), :), :), [], 2)];
	end	
	if ~isempty(ii_odom)
		XY = [XY; mat.odom_sample.states(i_, 1), mat.odom_sample.states(i_, 2)];
		UV = [UV; mat.odom_sample.states(i_, 3), mat.odom_sample.states(i_, 4)];
	end
	if ~isempty(UV)
		h_quiver = quiver(ax, XY(:, 1), XY(:, 2), UV(:, 1), UV(:, 2), 0, 'Color', 'r');
		tmp_handles = [tmp_handles, h_quiver];
	end

	xlim(ax, x_limits)
	ylim(ax, y_limits)
	daspect(ax, [1, 1, 1])
	title(ax, sprintf('t=%.2fs', t(i) - mat.odom_sample.t(1)))
	
    frames{i} = getframe(ax);
    
    pause(0.1)
	if isobject(fig) & ~isgraphics(fig)
		break
	end
	if rewind(fig)
		i = max([0, i - 20]);
	end
	if ~ishandle(fig)
		break
	end
end

writerObj = VideoWriter(videoname, 'Uncompressed AVI');
writerObj.FrameRate = 20;
open(writerObj);
for i = 1:nt
    if isempty(frames{i})
        break
    end
    writeVideo(writerObj, frames{i});
end
close(writerObj);


function h = plotcircle(ax, x, y, r, c1, c2)
if nargin == 5
	c2 = c1;
end
h = rectangle(ax, 'Position', [x - r, y - r, 2*r, 2*r], ...
	'Curvature',[1, 1], 'FaceColor', c1, 'EdgeColor', c2);


function C = colororder
C = [
	0 0.4470 0.7410	
	0.8500 0.3250 0.0980
	0.9290 0.6940 0.1250
	0.4940 0.1840 0.5560
	0.4660 0.6740 0.1880
	0.3010 0.7450 0.9330
	0.6350 0.0780 0.1840
];

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