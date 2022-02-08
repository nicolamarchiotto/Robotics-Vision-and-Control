function B = B_recursive_NewtonEulero(dh, q)
    n = dh.dof;
    B = zeros(n);
    I = eye(n);
    for i = 1:n
        bi = inv_dyn_recursive_NewtonEulero(dh, q, [0 0 0 0 0 0]', I(:,i), [0 0 0]', [0 0 0 0 0 0]');
        B(:,i) = bi;
    end    
end

