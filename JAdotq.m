%%
% Analytical Jacobian
% dh: dh table of the robot
% q: joints position

function dotx = JAdotq(dh,u)

    q=u(1:dh.dof);
    dotq=u(dh.dof:end);

%     T = sym(compute_transformation_matrix(0, 6, dh, q));
%     T = simplify(T);        
    
    T = compute_transformation_matrix(0, 6, dh, q);
    R = T(1:3, 1:3);
    
    theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    phi = atan2(R(2,3), R(1,3));
    psi = atan2(R(3,2), -R(3,1));
    
    Transformation = [0 -sin(phi) cos(phi)*sin(theta); ...
                      0  cos(phi) sin(phi)*sin(theta); ...
                      1  0        cos(theta)];
                  
    Ta = [eye(3) zeros(3); zeros(3) inv(Transformation)];
    
    Ja = double(Ta * Jacobian(dh, q));

    dotx = Ja*dotq;
end

