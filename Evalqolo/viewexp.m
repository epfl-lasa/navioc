function viewexp(mat)
hold on
tracksview(mat.tracked_persons)
odomview(mat.odom_sample)
occallview(mat.oc_call, 2.0)
ocreplyview(mat.oc_reply, 4.0)
daspect([1, 1, 1])

function tracksview(tracked_persons)
for j = 1:size(tracked_persons.states, 2)
	if any(tracked_persons.isdef(:, j))
		obj = plot(tracked_persons.states(tracked_persons.isdef(:, j), j, 1), ...
			tracked_persons.states(tracked_persons.isdef(:, j), j, 2));
		ii_def = find(tracked_persons.isdef(:, j));
		plot(tracked_persons.states(ii_def(1), j, 1), ...
			tracked_persons.states(ii_def(1), j, 2), 'Marker', '*', 'Color', obj.Color)
		plot(tracked_persons.states(ii_def(end), j, 1), ...
			tracked_persons.states(ii_def(end), j, 2), 'Marker', 'o', 'Color', obj.Color)
	end
end

function odomview(odom_sample)
plot(odom_sample.states(:, 1), odom_sample.states(:, 2), 'k', 'LineWidth', 2)
plot(odom_sample.states(1, 1), odom_sample.states(1, 2), 'k*')
plot(odom_sample.states(end, 1), odom_sample.states(end, 2), 'ko')

function occallview(oc_call, dt)
if isempty(oc_call.t)
    return
end
h = mean(oc_call.t(2:end) - oc_call.t(1:(end - 1)));
ii = 1:ceil(dt/h):length(oc_call.t);
plot(oc_call.x(ii, 1), oc_call.x(ii, 2), 'rs')
quiver(oc_call.x(ii, 1), oc_call.x(ii, 2), oc_call.x(ii, 3), oc_call.x(ii, 4), 'r')

function ocreplyview(oc_reply, dt)
if isempty(oc_reply.t)
    return
end
h = mean(oc_reply.t(2:end) - oc_reply.t(1:(end - 1)));
ii = 1:ceil(dt/h):length(oc_reply.t);
for i = ii
	plot(oc_reply.states(i, :, 1), oc_reply.states(i, :, 2), 'g')
end
