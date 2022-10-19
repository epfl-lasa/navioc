% Helper function to convert arrays of coordinates into indices.
function indices = coord2ind(coords,dims)

coords = coords - 1;
indices = ones(size(coords,1),1);
fac = 1;

for k=1:length(dims),
    indices = indices + coords(:,k)*fac;
    fac = fac*dims(k);
end;
