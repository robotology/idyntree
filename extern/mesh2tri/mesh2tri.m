function [F,V]=mesh2tri(X,Y,Z,tri_type)

% function [F,V]=mesh2tri(X,Y,Z,tri_type)
% ------------------------------------------------------------------------
%
% This function converts a regular mesh defined by X,Y and Z into a regular
% triangulation. The output is patch data (triangles) in the faces “F” and
% vertices “V” format. The quadrilateral mesh faces are converted to
% triangles by splitting the faces into triangles according to the setting
% tri_type:
%   tri_type ='f' -> forward slash division of quadrilateral
%   tri_type ='b' -> back slash division of quadrilateral
%   tri_type ='x' -> Cross division of quadrilateral
%
% The output coordinates "V" are in the form of V=[X(:),Y(:),Z(:)];
% For forward and back slash subdivision no extra coordinates are
% introduced and therefore the original meshgrid formatted coordinates can
% still be used for plotting, see examples below.
% For cross division extra points are created at the centre of each
% quadrilateral face using the mean of the input coordinates. The extra
% coordinates are the last prod(size(X)-1) points (e.g.
% V((numel(X)+1):end,:) )and can therefore be replaced by interpolated
% coordinates if desired, see example.
%
%
% %% EXAMPLE
% clear all; close all; clc;
%
% [X,Y] = meshgrid(linspace(-10,10,25));
% Z = sinc(sqrt((X/pi).^2+(Y/pi).^2));
%
% figure('units','normalized','Position',[0 0 1 1],'Color','w'); colordef('white');
% subplot(2,2,1);
% surf(X,Y,Z); hold on;
% axis tight; axis square; grid on; axis off; view(3); view(-30,70);
% title('Meshgrid','FontSize',20);
%
% [F,V]=mesh2tri(X,Y,Z,'f');
% C=V(:,3); C=mean(C(F),2);
% subplot(2,2,2);
% patch('Faces',F,'Vertices',V,'FaceColor','flat','CData',C); hold on;
% axis tight; axis square; grid on; axis off; view(3); view(-30,70);
% title('Forward slash','FontSize',20);
%
% [F,V]=mesh2tri(X,Y,Z,'b');
% C=V(:,3); C=mean(C(F),2);
% subplot(2,2,3);
% Example of using original meshgrid coordinates instead
% trisurf(F,X,Y,Z);
% axis tight; axis square; grid on; axis off; axis off; view(3); view(-30,70);
% title('Back slash','FontSize',20);
%
% [F,V]=mesh2tri(X,Y,Z,'x');
% Replace Z-coordinates of added points by interpolated values if desired
% IND=(numel(X)+1):size(V,1);
% ZI = interp2(X,Y,Z,V(IND,1),V(IND,2),'cubic');
% V(IND,3)=ZI;
%
% C=V(:,3); C=mean(C(F),2);
% subplot(2,2,4);
% patch('Faces',F,'Vertices',V,'FaceColor','flat','CData',C); hold on;
% axis tight; axis square; grid on; axis off; view(3); view(-30,70);
% title('Crossed','FontSize',20);
%
%
% Kevin Mattheus Moerman
% kevinmoerman@hotmail.com
% 15/07/2010
%------------------------------------------------------------------------

[J,I]=meshgrid(1:1:size(X,2)-1,1:1:size(X,1)-1);

switch tri_type
    case 'f'%Forward slash
        TRI_I=[I(:),I(:)+1,I(:)+1;  I(:),I(:),I(:)+1];
        TRI_J=[J(:),J(:)+1,J(:);   J(:),J(:)+1,J(:)+1];
        F = sub2ind(size(X),TRI_I,TRI_J);
    case 'b'%Back slash
        TRI_I=[I(:),I(:)+1,I(:);  I(:)+1,I(:)+1,I(:)];
        TRI_J=[J(:)+1,J(:),J(:);   J(:)+1,J(:),J(:)+1];
        F = sub2ind(size(X),TRI_I,TRI_J);
    case 'x'%Cross
        TRI_I=[I(:)+1,I(:);  I(:)+1,I(:)+1;  I(:),I(:)+1;    I(:),I(:)];
        TRI_J=[J(:),J(:);    J(:)+1,J(:);    J(:)+1,J(:)+1;  J(:),J(:)+1];
        IND=((numel(X)+1):numel(X)+prod(size(X)-1))';
        F = sub2ind(size(X),TRI_I,TRI_J);
        F(:,3)=repmat(IND,[4,1]);
        Fe_I=[I(:),I(:)+1,I(:)+1,I(:)]; Fe_J=[J(:),J(:),J(:)+1,J(:)+1];
        Fe = sub2ind(size(X),Fe_I,Fe_J);
        Xe=mean(X(Fe),2); Ye=mean(Y(Fe),2);  Ze=mean(Z(Fe),2);
        X=[X(:);Xe(:)]; Y=[Y(:);Ye(:)]; Z=[Z(:);Ze(:)];
end

V=[X(:),Y(:),Z(:)];

end
