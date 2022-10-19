% Helper function to convert indices into arrays of coordinates.
function coords = ind2coord(indices,dims)

% Make zero-indexed.
indices = indices-1;
coords = zeros(size(indices,1),length(dims));

for k=1:length(dims),
    coords(:,k) = mod(indices,dims(k))+1;
    indices = floor(indices/dims(k));
end;
