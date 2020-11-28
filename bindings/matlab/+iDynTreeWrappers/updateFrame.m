function updateFrame(frame, newTransform)

%% updateFrame 
% updateFrame updates a 3D frame added with the plotFrame function
% Inputs:
%    - frame: The struct output by plotFrame
%    - newTransform : The new 3D transformation (4x4 matrix)
%
% Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.

R = newTransform(1:3, 1:3);

or = newTransform(1:3, 4);

p = [or, or, or] + frame.axisDimension * R;

set(frame.x, 'XData', [or(1) p(1,1)], 'YData', [or(2) p(2,1)], 'ZData', [or(3) p(3,1)]);
set(frame.y, 'XData', [or(1) p(1,2)], 'YData', [or(2) p(2,2)], 'ZData', [or(3) p(3,2)]);
set(frame.z, 'XData', [or(1) p(1,3)], 'YData', [or(2) p(2,3)], 'ZData', [or(3) p(3,3)]);

p_text = [or, or, or] + frame.axisDimension * 1.25 * R;

set(frame.labels.x, 'Position', p_text(1:3,1));
set(frame.labels.y, 'Position', p_text(1:3,2));
set(frame.labels.z, 'Position', p_text(1:3,3));

end

