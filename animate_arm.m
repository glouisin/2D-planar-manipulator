function animate_arm(Q,L)
  figure; hold on; axis equal; grid on;
    R = sum(L);
    axis([-R R -R R]);
    h     = plot(0, 0, 'o-', 'LineWidth', 3, 'MarkerSize', 8);
    trace = plot(NaN, NaN, 'r--');
    traj = [];
    for k = 1:size(Q, 2)
        [r, P] = forward_kinematic(Q(1,k), Q(2,k), L(1), L(2));
        traj = [traj r];
        set(h, 'XData', P(1,:), 'YData', P(2,:));
        set(trace, 'XData', traj(1,:), 'YData', traj(2,:));
        drawnow;
        pause(0.02);
    end
end