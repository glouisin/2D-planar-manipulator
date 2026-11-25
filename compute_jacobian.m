    function Jv = compute_jacobian(THETA, L)
        [R,P] = forward_kinematics(THETA,L);
        p1 = P(:,1); % warning, p0 is the reference point
        p2 = P(:,2);
        p1_eff = R - p1;
        p2_eff = R - p2;
        Jv1 = [-p1_eff(2); p1_eff(1)]; % Jvi = [-pye_i ; pxe_i]
        Jv2 = [-p2_eff(2); p2_eff(1)];
        Jv = [Jv1 Jv2];
    end

