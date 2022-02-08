% Homework 2 -- Create a 3D cloud of points from a range image
clear all; close all; clc;

depth = int32(imread('0000001-000000000000.png'));
rgb = int32(imread('0000001-000000000000.jpg'));


[row, column] = size(depth);

% The focal length fu and fv is 525 for both, data from  PrimeSense Carmine cameras

fu = 525;
fv = 525;

% Principal point 
uo = 319.5;
vo = 239.5;

figure(1);
colormap jet;
imagesc(depth);

indexmap = zeros(row, column);
pointCloud = [];
colors = [];
index = 1;

x_im = zeros(size(depth));
y_im = zeros(size(depth));

for u = 1:1:row
    for v = 1:1:column
        z = depth(u,v);
        
        % Discard the range map too close 
        if z == 0
            continue;
        end
        
        % Inverse of the formula on the slide
        x_im = -((depth(u,v)*(u - uo))/fu);
        y_im = -((depth(u,v)*(v - vo))/fv);
        
        % Collecting cloud of points
        pointCloud = [pointCloud; [x_im y_im -z]];
        % Collecting colors
        colors = [colors; rgb(u,v)];
        
        % Save index of cloud, needs for next assignment 
        indexmap(u, v) = index;
        index = index +1;   
    end
end

figure(2)
plot3(pointCloud(:,1), pointCloud(:,2), pointCloud(:,3),'r.');
%% Homowork 3 -- Mesh reconstruction from range image

faces = [];
t = 7;

for u = 1:row - 1
    for v = 1: column - 1
        z = depth(u,v);
        
        % Discard the range map too close -- We avoid to control the point A 
        if z == 0
            continue
        end
       
       % Collect the indeces
       v_or = v + 1;
       u_ver = u + 1;
       in_diag_u = u_ver;
       in_diag_v = v_or;
       
       % Create the 4 verteces of the rectangle 
        A = indexmap(u,v);
        B = indexmap(u_ver,v);
        C = indexmap(u, v_or);
        D = indexmap(in_diag_u, in_diag_v);
        
        % We don't have both the upper and lower triangle
        if B == 0 || C == 0 
            continue
        end
        
        % we could have the upper triangle
        if D == 0
            seg1 = point2point_distance(double(pointCloud(A,:)), double(pointCloud(B,:)));
            seg2 = point2point_distance(double(pointCloud(A,:)), double(pointCloud(C,:)));
            seg5 = point2point_distance(double(pointCloud(B,:)), double(pointCloud(C,:)));
            
            if seg1 < t || seg2< t || seg5 < t
                faces = [faces; [A B C]];
            end
            
        else
            
            % we could have both the upper and lower triangle
            seg1 = point2point_distance(double(pointCloud(A,:)), double(pointCloud(B,:)));
            seg2 = point2point_distance(double(pointCloud(A,:)), double(pointCloud(C,:)));
            seg3 = point2point_distance(double(pointCloud(B,:)), double(pointCloud(D,:)));
            seg4 = point2point_distance(double(pointCloud(C,:)), double(pointCloud(D,:)));
            seg5 = point2point_distance(double(pointCloud(B,:)), double(pointCloud(C,:)));
        
            if seg1 < t || seg2< t || seg5 < t
                faces = [faces; [A B C]];
            elseif seg3 < t || seg4< t || seg5 < t
                faces = [faces; [B C D]];
            end
        end
        
    end
    
end


exportMeshToPly(pointCloud, faces , colors, 'mesh_es1_3');

