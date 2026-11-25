function [r, p] = forward_kinematic(THETA, L)
        theta_1 = THETA(1);
        theta_2 = THETA(2);
        tetha_12 = theta_1 + theta_2;
        p0 = [0;0]; % origin point
        p1 = l1*[cos(theta_1);sin(theta_1)]; % 1st joint position
        p2 = l2*[cos(tetha_12);sin(tetha_12)] + p1;
        p = [p0 p1 p2];
        x_eff = p2(1);
        y_eff = p2(2);
        r = [x_eff; y_eff];
end


