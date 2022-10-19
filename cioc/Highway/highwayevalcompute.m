% Evaluate statistics on a single frame.
function results = highwayevalcompute(evalstruct,x,t,mdp_data)

% Allocate.
results = zeros(4,1);

% Add to mean speed in KPH.
results(1) = x(4)*1000/evalstruct.T;

% Compute car polygon.
rm = [cos(x(3)) sin(x(3)); -sin(x(3)) cos(x(3))];
agentcar = bsxfun(@plus,x(1:2),(rm*(evalstruct.carpoly'))');

for j=1:length(mdp_data.cars),
    % Get car configuration.
    carx = mdp_data.cars{j}.x(t,:);

    % Create polygons.
    rm = [cos(carx(3)) sin(carx(3)); -sin(carx(3)) cos(carx(3))];
    car = bsxfun(@plus,carx(1:2),(rm*(evalstruct.carpoly'))');
    carback = bsxfun(@plus,carx(1:2),(rm*(evalstruct.carbackpoly'))');
    carfront = bsxfun(@plus,carx(1:2),(rm*(evalstruct.carfrontpoly'))');

    % Check for cases.
    if intersectpoly(agentcar,car),
        results(2) = results(2) + 1;
    end;
    if intersectpoly(agentcar,carback),
        results(3) = results(3) + 1;
    end;
    if intersectpoly(agentcar,carfront),
        results(4) = results(4) + 1;
    end;
end;

end

function check = intersectpoly(poly1,poly2)
    % Check if any lines in the two polygons intersect.
    for l1=1:(size(poly1,1)-1),
        for l2=1:(size(poly2,1)-1),
            p1 = poly1(l1,:);
            p2 = poly1(l1+1,:);
            p3 = poly2(l2,:);
            p4 = poly2(l2+1,:);
            % Check for intersection.
            den = (p4(2)-p3(2))*(p2(1)-p1(1)) - (p4(1)-p3(1))*(p2(2)-p1(2));
            if den == 0,
                continue;
            end;
            u1 = ((p4(1)-p3(1))*(p1(2)-p3(2)) - (p4(2)-p3(2))*(p1(1)-p3(1)))/den;
            u2 = ((p2(1)-p1(1))*(p1(2)-p3(2)) - (p2(2)-p1(2))*(p1(1)-p3(1)))/den;
            if u1 >= 0 && u1 <= 1 && u2 >= 0 && u2 <= 1,
                check = 1;
                return;
            end;
        end;
    end;
    check = 0;
end
