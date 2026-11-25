function animate_arm(Q,L, r_d)
    figure; hold on; axis equal; grid on;
    R = sum(L);
    axis([-R R -R R]);
    if nargin >= 3     % 3rd arg for IK methods
        plot(r_d(1), r_d(2), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
    end
    h     = plot(0, 0, 'o-', 'LineWidth', 3, 'MarkerSize', 8);
    trace = plot(NaN, NaN, 'r--');
    traj = [];
    for k = 1:size(Q, 2)
        [r, P] = forward_kinematics(Q(:,k),L);

        traj = [traj r];
        set(h, 'XData', P(1,:), 'YData', P(2,:));
        set(trace, 'XData', traj(1,:), 'YData', traj(2,:));
        drawnow;
        pause(0.02);
    end
end
