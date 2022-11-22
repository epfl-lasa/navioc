% robot starts by avoiding a pedestrian
% then enters a lane
% where at some point, a woman ahead notices the robot and increases her distance to it
% while the robot keeps moving more or less straight
% playexp(loadexp(getfpaths("crowd", "navioc", 1)), 39)

% a pedestrian changes course as the robot starts moving in the back
% the robot passes by two pedestrians who adapt more to it than vice versa
% playexp(loadexp(getfpaths("crowd", "navioc", 2)), 34)

% the robot oscillates between avoiding a pedestrian on the left or on the right
% and the situation is resolved by a quick change of course by the pedestrian
% then, the robot attempts to avoid a standing pedestrian on the left, but
% as it approaches, it perceives another pedestrian, and so it converges to
% passing in between them, even though there is not enough space, such that it collides with
% the first pedestrian's foot (also, the other one is duplicated, and thus creates higher cost) 
% playexp(loadexp(getfpaths("crowd", "navioc", 3)), 41)

% the robot plans to avoid a pedestrian, but the pedestrian changes course much quicker than the robot
% playexp(loadexp(getfpaths("crowd", "navioc", 4)), 48)

% I pass by the robot which avoids me less than I avoid it
% playexp(loadexp(getfpaths("crowd", "navioc", 5)), 50)