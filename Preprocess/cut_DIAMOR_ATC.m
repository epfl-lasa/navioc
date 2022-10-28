function batches = cut_DIAMOR_ATC(dt_margin)

subdatasets = cell(1, 26);

subdatasets{1} = struct(...
    'tracks_name', 'person_DIAMOR-1_all.csv', ...
    'info_name', 'groups_DIAMOR-1.dat');

subdatasets{2} = struct(...
    'tracks_name', 'person_DIAMOR-2_all.csv', ...
    'info_name', 'groups_DIAMOR-2.dat');

digit_seq = [0, 2, 5, 9];
k = 2;
for i = 1:6
	info_name = sprintf('groups_ATC-%i.dat', i);
	for j = 1:4
		tracks_name = sprintf('person_ATC-%i_1%i00.csv', i, digit_seq(j));

        k = k + 1;
        subdatasets{k} = struct(...
            'tracks_name', tracks_name, ...
            'info_name', info_name);
    end
end
        
batches = {};
for s = 1:length(subdatasets)
    tracks_name = subdatasets{s}.tracks_name;
    info_name = subdatasets{s}.info_name;
 
    fprintf('Parsing tracks file %s ...\n', tracks_name);
    success = false;
    while ~success
        try
            tracks = importdata("diamor-atc", tracks_name, info_name);
            success = true;
        catch ME
            fprintf(2, ME.message)
            fprintf('Retrying in 5 seconds ...\n');
            pause(5)
        end
    end
        
    fprintf('Cutting around wheelchairs ...\n');
    [windows, slice_indices] = wheelchairwindows(tracks, dt_margin);
        
    new_batches = cell(1, length(slice_indices));
    for k = 1:length(slice_indices)
        new_batches{k} = struct(...
            'source', tracks_name, ...
            'tracks', {tracks(slice_indices{k})}, ...
            'window', windows(:, k) ...
        );
    end

    batches = [batches, new_batches];
end