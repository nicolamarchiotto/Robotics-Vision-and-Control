function C = C_recursive_NewtonEulero(dh, q, qd)
    
    n = dh.dof;
    
    C = zeros(n, n);
    Csq = zeros(n, n);
    
    dh2 = dh;
    %%dh2 no friction
    
    for i=1:n
       QD = zeros(n,1);
       QD(i) = 1;
       tau = inv_dyn_recursive_NewtonEulero(dh2, q, QD, zeros(n,1), [0 0 0]', [0 0 0 0 0 0]');
       Csq(:,i) = Csq(:,i) + tau;
    end
    
    for i=1:n
        for k=i+1:n
            QD = zeros(n,1);
            QD(i) = 1;
            QD(k) = 1;
            tau = inv_dyn_recursive_NewtonEulero(dh2, q, QD, zeros(n, 1), [0 0 0]', [0 0 0 0 0 0]');
            C(:,k) = C(:,k) + (tau - Csq(:,k) - Csq(:,i)) * qd(i)/2;
            C(:,i) = C(:,i) + (tau - Csq(:,k) - Csq(:,i)) * qd(k)/2;
        end
    end
    C = C + Csq * diag(qd);

end