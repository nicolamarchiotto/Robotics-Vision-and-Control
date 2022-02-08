%% Geometric Jacobian derivative * q_dot

function J_dot = Jacobian_g_dot(dh, q, qd)

    n = length(dh.d);
    
    for i=1:n
        T = compute_transformation_matrix(i-1, i, dh, q);
        Q{i} = T(1:3, 1:3);
        a{i} = T(1:3, 4);
    end

    P{1} = Q{1};
    e{1} = [0 0 1]';
    for i=2:n
        P{i} = P{i-1}*Q{i};
        e{i} = P{i}(:,3);
    end

    % step 1
    w{1} = qd(1)*e{1};
    for i=1:(n-1)
        w{i+1} = qd(i+1)*[0 0 1]' + Q{i}'*w{i};
    end

    % step 2
    ed{1} = [0 0 0]';
    for i=2:n
        ed{i} = cross(w{i}, e{i});
    end

    % step 3
    rd{n} = cross( w{n}, a{n});
    for i=(n-1):-1:1
        rd{i} = cross(w{i}, a{i}) + Q{i}*rd{i+1};
    end

    r{n} = a{n};
    for i=(n-1):-1:1
        r{i} = a{i} + Q{i}*r{i+1};
    end

    ud{1} = cross(e{1}, rd{1});
    for i=2:n
        ud{i} = cross(ed{i}, r{i}) + cross(e{i}, rd{i});
    end

    % step 4
    %  swap ud and ed
    v{n} = qd(n)*[ud{n}; ed{n}];
    for i=(n-1):-1:1
        Ui = blkdiag(Q{i}, Q{i});
        v{i} = qd(i)*[ud{i}; ed{i}] + Ui*v{i+1};
    end

    J_dot = double(v{1});

end