function [Q_hist, err_hist] = inverse_kinematics_inverse(q0, r_d,L, k, dt, epsilon, N)
    % inverse kinematics with inverse Jacobian method
    q = q0;
    Q_hist = q0; % keeps history for plotting
    err_hist = [];
    for i = 1:N
        r = forward_kinematics(q,L);
        v = k*(r_d -r);
        err = norm(r_d -r);
        err_hist = [err_hist err];
        if err < epsilon % threshold reached
            break 
        end
        Jv = compute_jacobian(q, L);
        qpoint = Jv\v;
        q = q + qpoint*dt;
        Q_hist = [Q_hist q] ;
    end
end


