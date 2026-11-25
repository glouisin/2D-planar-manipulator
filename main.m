%% ============== Q1 ===================
% Lenght of each joint
L = [1; 1];
% Space of angles
N = 1000;
Q = [linspace(0, pi/2,N); linspace(0,pi/2,N)];
animate_arm(Q,L)
%% ============== Q2 ===================
L = [1;1]
q0 = [pi/2; pi/2]
N = 50
% test for 1 value
Jv = compute_jacobian(q0, L)
v_wanted = [1;1] % wanted speed direction
dt = 0.01
qpoint = Jv\v_wanted
r_new = forward_kinematics(q0 + dt*qpoint, L) % position after dt*qpoint step 
[r_origin, p_origin] = forward_kinematics (q0, L) % origin position of the effector
v_true = (r_new - r_origin)/dt
err = norm(v_wanted - v_true) % error << 1
% test for ten values
thetas = (0:9)*pi/5;
colors = lines(10);
figure; hold on; axis equal; grid on;
axis([-2 2 -2 2]);  
arm = plot(p_origin(1,:), p_origin(2,:), 'ko-', 'LineWidth', 3, 'MarkerSize', 8);
for theta_i = 1:length(thetas)
    v_wanted_i = [cos(thetas(theta_i)); sin(thetas(theta_i))];
    q_i = q0;
    T = zeros(2, N+1);
    T(:,1) = r_origin;
    h_traj = plot(r_origin(1), r_origin(2), '-', ...
                  'Color', colors(theta_i,:), 'LineWidth', 2);  
    for k = 1:N
        Jv_i = compute_jacobian(q_i, L);
        qpoint_i = Jv_i\v_wanted_i;
        q_i = q_i + dt*qpoint_i;
        [T(:,k+1), P_i] = forward_kinematics(q_i, L);
        % animation
        set(arm,  'XData', P_i(1,:),     'YData', P_i(2,:));      
        set(h_traj, 'XData', T(1,1:k+1),   'YData', T(2,1:k+1));    
        drawnow; pause(0.02);
    end
    
end

xlabel('x'); ylabel('y');
title('End-effector trajectories for 10 desired directions');

%% ============== Q3 ===================
L   = [1; 1];
q0  = [pi/6; pi/3];
r_d = [-1; 0];  % target position
N = 1000;   % iterations
k = 1; % gain (proportional)
eta = 1; % gain
epsilon = 1e-4; %convergence treshold
dt = 0.05
% vicous coefficients 
c1 = 1;
c2 = 1;
C = [c1,c2]  

% inverse method
[Q_inv, err_inv] = inverse_kinematics_inverse(q0, r_d, L, k, dt, epsilon, N);

q_final = Q_inv(:, end)
r_check = forward_kinematics(q_final, L)     %  r_d
nb_iter = length(err_inv)

animate_arm(Q_inv, L, r_d);


%transpose method

[Q_tr, err_tr] = inverse_kinematics_transpose(q0, r_d, L, eta,C,dt, epsilon, N);

q_final_tr = Q_tr(:, end)
r_check_tr = forward_kinematics(q_final_tr, L)   
nb_iter_tr = length(err_tr)

animate_arm(Q_tr, L, r_d);

figure;
semilogy(err_inv, 'r', 'LineWidth', 2);
hold on;
semilogy(err_tr,  'b', 'LineWidth', 2);
grid on;
legend('Jacobian inverse', 'Jacobian transpose');
xlabel('iteration'); ylabel('|r_d - r|');


%% ============== Q4 ===================
%Testing singularity. 
L   = [1; 1];
q0  = [pi/2; pi/2];
r_d = [2; 3];  % target position
N = 1000;   % iterations
k = 1; % gain (proportional)
eta = 1; % gain
epsilon = 1e-4; %convergence treshold
dt = 0.05
% vicous coefficients 
c1 = 1;
c2 = 1;
C = [c1,c2]  

% inverse method
[Q_inv, err_inv] = inverse_kinematics_inverse(q0, r_d, L, k, dt, epsilon, N);

q_final = Q_inv(:, end)
r_check = forward_kinematics(q_final, L)     %  r_d
nb_iter = length(err_inv)

animate_arm(Q_inv, L, r_d);


%transpose method

[Q_tr, err_tr] = inverse_kinematics_transpose(q0, r_d, L, eta,C,dt, epsilon, N);

q_final_tr = Q_tr(:, end)
r_check_tr = forward_kinematics(q_final_tr, L)   
nb_iter_tr = length(err_tr)

animate_arm(Q_tr, L, r_d);

figure;
semilogy(err_inv, 'r', 'LineWidth', 2);
hold on;
semilogy(err_tr,  'b', 'LineWidth', 2);
grid on;
legend('Jacobian inverse', 'Jacobian transpose');
xlabel('iteration'); ylabel('|r_d - r|');

%% BONUS - 3D error landscape (Made with the help of Claude AI)
t = linspace(-pi, pi, 120);
[T1, T2] = meshgrid(t, t);
E = zeros(size(T1));
for j = 1:numel(T1)
    E(j) = norm(r_d - forward_kinematics([T1(j); T2(j)], L));
end
figure; hold on; grid on;
surf(T1, T2, E, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
% lift each IK path onto the surface: z = |r_d - f(q)|
z_inv = arrayfun(@(i) norm(r_d - forward_kinematics(Q_inv(:,i), L)), 1:size(Q_inv,2));
z_tr  = arrayfun(@(i) norm(r_d - forward_kinematics(Q_tr(:,i),  L)), 1:size(Q_tr,2));
plot3(Q_inv(1,:), Q_inv(2,:), z_inv + 0.05, 'r-', 'LineWidth', 2.5);
plot3(Q_tr(1,:),  Q_tr(2,:),  z_tr  + 0.05, 'b-', 'LineWidth', 2.5);
xlabel('\theta_1 [rad]'); ylabel('\theta_2 [rad]'); zlabel('|r_d - f(q)|');
legend('error surface', 'Jacobian inverse', 'Jacobian transpose');
colorbar; view(3);
% note: for r_d = [2;3] (Q4) the inverse path diverges in joint space;
% run the Q3 section first to see the clean descent.

%% BONUS - Error contour + IK paths (joint space)
t = linspace(-pi, pi, 200);
[T1, T2] = meshgrid(t, t);
E = zeros(size(T1));
for j = 1:numel(T1)
    E(j) = norm(r_d - forward_kinematics([T1(j); T2(j)], L));
end

figure; hold on; axis equal; grid on;
contour(T1, T2, E, 25);                          % 25 levels
colorbar;
plot(Q_inv(1,:), Q_inv(2,:), 'r-',  'LineWidth', 2.5);
plot(Q_tr(1,:),  Q_tr(2,:),  'b-',  'LineWidth', 2.5);
plot(q0(1), q0(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
plot(Q_inv(1,end), Q_inv(2,end), 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'y');
xlabel('\theta_1 [rad]'); ylabel('\theta_2 [rad]');
legend('|r_d - f(q)|', 'Jacobian inverse', 'Jacobian transpose', 'start', 'solution');
title('IK paths in joint space');
xlim([-pi pi]); ylim([-pi pi]);