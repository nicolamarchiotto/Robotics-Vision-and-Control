function [distance] = point2point_distance(P1, P2)
            
            distance = sqrt( (P1(1)- P2(1))^2 + (P1(2)- P2(2))^2 + (P1(3)- P2(3))^2 );
end

