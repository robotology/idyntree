function frame = plotFrame(transform, axisDimension, lineWidth)

%% plotFrame 
% plotFrame adds a 3D frame in the current figure
% Inputs:
%    - transform: The transform of the frame (4x4 matrix)
%    - axisDimension: The dimension of the axes
%    - lineWidth: The linewidth of the axes
% Outputs:
%    - A struct that can be used with updateFrame to update the
%    visualization
%
% Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.

hold on
parent=gca;

frame = struct();

R = transform(1:3, 1:3);

or = transform(1:3, 4);

p = [or, or, or] + axisDimension * R;

frame.axisDimension = axisDimension;
frame.lineWidth = lineWidth;

frame.x = plot3(parent, [or(1) p(1,1)], [or(2) p(2,1)], [or(3) p(3,1)], 'r', 'linewidth', lineWidth);
frame.y = plot3(parent, [or(1) p(1,2)], [or(2) p(2,2)], [or(3) p(3,2)], 'g', 'linewidth', lineWidth);
frame.z = plot3(parent, [or(1) p(1,3)], [or(2) p(2,3)], [or(3) p(3,3)], 'b', 'linewidth', lineWidth);

p_text = [or, or, or] + axisDimension * 1.25 * R;

frame.labels = struct();

frame.labels.x = text(parent, p_text(1,1), p_text(2,1), p_text(3,1), 'x', 'color', 'r');
frame.labels.y = text(parent, p_text(1,2), p_text(2,2), p_text(3,2), 'y', 'color', 'g');
frame.labels.z = text(parent, p_text(1,3), p_text(2,3), p_text(3,3), 'z', 'color', 'b');

end

