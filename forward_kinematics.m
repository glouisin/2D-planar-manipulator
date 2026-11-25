function [r, p] = forward_kinematics(THETA, L)
        theta_1 = THETA(1);
        theta_2 = THETA(2);
        l1 = L(1);
        l2 = L(2);
        tetha_12 = theta_1 + theta_2;
        p0 = [0;0]; % origin point
        p1 = l1*[cos(theta_1);sin(theta_1)]; % 1st joint position
        p2 = l2*[cos(tetha_12);sin(tetha_12)] + p1; % 2nd joint
        p = [p0 p1 p2];
        x_eff = p2(1);
        y_eff = p2(2);
        r = [x_eff; y_eff];
end


