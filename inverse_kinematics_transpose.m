function [Q_hist, err_hist] = inverse_kinematics_transpose(q0, r_d,L, eta,C, dt, epsilon, N)
    % inverse kinematics with transposed Jacobian method
    q = q0;
    c1 = C(1);
    c2 = C(2);
    K = [1/c1 0; 0 1/c2]
    Q_hist = q0; % keeps history for plotting
    err_hist = [];
    for i = 1:N
        r = forward_kinematics(q,L);
        F = eta*(r_d -r);
        err = norm(r_d -r);
        err_hist = [err_hist err];
        if err < epsilon % threshold reached
            break 
        end
        Jv = compute_jacobian(q, L);
        tau = Jv'* F;
        
       
        qpoint = K*tau;
        q = q + qpoint*dt;
        Q_hist = [Q_hist q] ;
    end
end


