function [angle]=angleBetweenLines(direction1,direction2)
    if(size(direction1,1)~=3 && size(direction1,2)~=1)
        error("All points given as input must have dimension 3*1")
    end
    if(size(direction2,1)~=3 && size(direction2,2)~=1)
        error("All points given as input must have dimension 3*1")
    end
    angle=acosd(dot(direction1,direction2)/(norm(direction1)*norm(direction2)));
end