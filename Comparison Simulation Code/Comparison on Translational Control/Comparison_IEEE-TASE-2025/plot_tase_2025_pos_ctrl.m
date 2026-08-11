%% Simulation results of the follower desired-position distributed observer,
%% external disturbance observer, and translational controller

%% The controller is designed based on the IEEE TASE (2025) paper:
%% <L1 Adaptive Control-Based Formation Tracking of Multiple Quadrotors 
%% Without Linear Velocity Feedback Under Unknown Disturbances>

%% Execute <init_tase_2025_pos_ctrl.m> for initialization
%% Execute <sim_tase_2025_pos_ctrl.slx> to load the simulation model
%% Execute <plot_tase_2025_pos_ctrl.m> to visualize the simulation results
%% of the translational controller and disturbance observer

clc;
close all;

linestyle_vec = {'--', '-.', ':'};

color_vec = [252, 170, 103; ...
                        189, 30, 30; ...
                        124, 187, 0; ...
                        54, 195, 201; ...
                        0, 70, 222; ...
                        0, 0, 0] ./ 255;

%% Extract simulation data
NF = 5;
tLen = length(tout);

%%%% Estimation errors of the position/velocity distributed observer
ksi_obs_err_x = zeros(NF,tLen);
ksi_obs_err_y = zeros(NF,tLen);
ksi_obs_err_z = zeros(NF,tLen);
eta_obs_err_x = zeros(NF,tLen);
eta_obs_err_y = zeros(NF,tLen);
eta_obs_err_z = zeros(NF,tLen);

%%%% Position outputs
ksi_fl_x = zeros(NF,tLen);
ksi_fl_y = zeros(NF,tLen);
ksi_fl_z = zeros(NF,tLen);

%%%% Velocity outputs
eta_fl_x = zeros(NF,tLen);
eta_fl_y = zeros(NF,tLen);
eta_fl_z = zeros(NF,tLen);

%%%% Position estimation outputs of the differentiator
ksi_diff_x = zeros(NF,tLen);
ksi_diff_y = zeros(NF,tLen);
ksi_diff_z = zeros(NF,tLen);

%%%% Velocity estimation outputs of the differentiator
eta_diff_x = zeros(NF,tLen);
eta_diff_y = zeros(NF,tLen);
eta_diff_z = zeros(NF,tLen);

%%%% Position tracking errors
ksi_err_fl_x = zeros(NF,tLen);
ksi_err_fl_y = zeros(NF,tLen);
ksi_err_fl_z = zeros(NF,tLen);

%%%% Velocity tracking errors
eta_err_fl_x = zeros(NF,tLen);
eta_err_fl_y = zeros(NF,tLen);
eta_err_fl_z = zeros(NF,tLen);

%%%% Custom disturbances defined in the simulation
dp_x = zeros(NF,tLen);
dp_y = zeros(NF,tLen);
dp_z = zeros(NF,tLen);

%%%% Estimated disturbance values
dp_hat_x = zeros(NF,tLen);
dp_hat_y = zeros(NF,tLen);
dp_hat_z = zeros(NF,tLen);

%%%% Disturbance estimation errors
dp_tilt_x = zeros(NF,tLen);
dp_tilt_y = zeros(NF,tLen);
dp_tilt_z = zeros(NF,tLen);

%%%% Correct data format
if size(tout,2) == 1
    tout = tout';
    u0 = u0';
    eta0 = eta0';
    ksi0 = ksi0';
end

%%%% Extract simulation data
for tt = 1:tLen
    for iiFF = 1:NF
        ksi_obs_err_x(iiFF,tt) = ksi_est_err(1,iiFF,tt);
        ksi_obs_err_y(iiFF,tt) = ksi_est_err(2,iiFF,tt);
        ksi_obs_err_z(iiFF,tt) = ksi_est_err(3,iiFF,tt);
        eta_obs_err_x(iiFF,tt) = eta_est_err(1,iiFF,tt);
        eta_obs_err_y(iiFF,tt) = eta_est_err(2,iiFF,tt);
        eta_obs_err_z(iiFF,tt) = eta_est_err(3,iiFF,tt);
        ksi_fl_x(iiFF,tt) = ksi_i_vec(1,iiFF,tt);
        ksi_fl_y(iiFF,tt) = ksi_i_vec(2,iiFF,tt);
        ksi_fl_z(iiFF,tt) = ksi_i_vec(3,iiFF,tt);
        eta_fl_x(iiFF,tt) = eta_i_vec(1,iiFF,tt);
        eta_fl_y(iiFF,tt) = eta_i_vec(2,iiFF,tt);
        eta_fl_z(iiFF,tt) = eta_i_vec(3,iiFF,tt);
        ksi_diff_x(iiFF,tt) = pos_diff_hat_vec(1,iiFF,tt);
        ksi_diff_y(iiFF,tt) = pos_diff_hat_vec(2,iiFF,tt);
        ksi_diff_z(iiFF,tt) = pos_diff_hat_vec(3,iiFF,tt);
        eta_diff_x(iiFF,tt) = vel_diff_hat_vec(1,iiFF,tt);
        eta_diff_y(iiFF,tt) = vel_diff_hat_vec(2,iiFF,tt);
        eta_diff_z(iiFF,tt) = vel_diff_hat_vec(3,iiFF,tt);
        ksi_err_fl_x(iiFF,tt) = ksi_track_err_vec(1,iiFF,tt);
        ksi_err_fl_y(iiFF,tt) = ksi_track_err_vec(2,iiFF,tt);
        ksi_err_fl_z(iiFF,tt) = ksi_track_err_vec(3,iiFF,tt);
        eta_err_fl_x(iiFF,tt) = eta_track_err_vec(1,iiFF,tt);
        eta_err_fl_y(iiFF,tt) = eta_track_err_vec(2,iiFF,tt);
        eta_err_fl_z(iiFF,tt) = eta_track_err_vec(3,iiFF,tt);
        dp_x(iiFF,tt) = dp_vec(1,iiFF,tt);
        dp_y(iiFF,tt) = dp_vec(2,iiFF,tt);
        dp_z(iiFF,tt) = dp_vec(3,iiFF,tt);
        dp_hat_x(iiFF,tt) = dp_hat_vec(1,iiFF,tt);
        dp_hat_y(iiFF,tt) = dp_hat_vec(2,iiFF,tt);
        dp_hat_z(iiFF,tt) = dp_hat_vec(3,iiFF,tt);
        dp_tilt_x(iiFF,tt) = dp_tilt_vec(1,iiFF,tt);
        dp_tilt_y(iiFF,tt) = dp_tilt_vec(2,iiFF,tt);
        dp_tilt_z(iiFF,tt) = dp_tilt_vec(3,iiFF,tt);
    end
end

t0 = 0;
t1 = 10;
t12 = 20;
t2 = 30;
del_t = 0.1;

%% Plot the estimation errors of the practical fixed-time distributed state observers (PFxTDSO)
%% Position observation errors with a zoomed-in inset

figure(1)

subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, ksi_obs_err_x(iiFF,:), 'color', color_vec(id_color,:), ...
                            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
            {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
             'interpreter','latex','location','best','NumColumns',5, ...
             'box','off','color','none');
box on;
xlabel('time (s)');
ylabel('$$\it \tilde{p}_{i,x}^{d} \rm (m)$$','interpreter','latex');

% 按时间分段将背景填充
y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, ksi_obs_err_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('time (s)');
ylabel('$$\it \tilde{p}_{i,y}^{d} \rm (m)$$','interpreter','latex');

% 按时间分段将背景填充
y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, ksi_obs_err_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
set(gca, 'ylim', [-0.2, 0.2]);
box on;
xlabel('time (s)');
ylabel('$$\it \tilde{p}_{i,z}^{d} \rm (m)$$','interpreter','latex');

% 按时间分段将背景填充
y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

%%%% 子图1/2 -- 位置X估计误差局部放大图
ax1_1 = axes('Position', [0.2, 0.8, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 0.1));
[~, id_end] = min(abs(tout - 0.3));
axes(ax1_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ksi_obs_err_x(iiFF,id_start:id_end), 'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end

ax1_2 = axes('Position', [0.5, 0.8, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 1.6));
[~, id_end] = min(abs(tout - 2.0));
axes(ax1_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_obs_err_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end

%%%% 子图3/4 -- 位置Y估计误差局部放大图
ax2_1 = axes('Position', [0.2, 0.5, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 0.1));
[~, id_end] = min(abs(tout - 0.3));
axes(ax2_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ksi_obs_err_y(iiFF,id_start:id_end), 'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end

ax2_2 = axes('Position', [0.5, 0.5, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 1.6));
[~, id_end] = min(abs(tout - 2.0));
axes(ax2_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_obs_err_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end

%%%% 子图5 -- 位置Z估计误差局部放大图
ax3_1 = axes('Position', [0.5, 0.2, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 1.6));
[~, id_end] = min(abs(tout - 2.0));
axes(ax3_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_obs_err_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end

%% Plot the estimation errors of the practical fixed-time distributed state observers (PFxTDSO)
%% Velocity observation errors with a zoomed-in inset

figure(2)
subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, eta_obs_err_x(iiFF,:), 'color', color_vec(id_color,:), ...
                            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
            {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
             'interpreter','latex','location','best','NumColumns',5, ...
             'box','off','color','none');
box on;
xlabel('time (s)');
ylabel('$$\it \tilde{v}_{i,x}^{d} \rm (m/s)$$','interpreter','latex');

% 按时间分段将背景填充
y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, eta_obs_err_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('time (s)');
ylabel('$$\it \tilde{v}_{i,y}^{d} \rm (m/s)$$','interpreter','latex');

% 按时间分段将背景填充
y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, eta_obs_err_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('time (s)');
ylabel('$$\it \tilde{v}_{i,z}^{d} \rm (m/s)$$','interpreter','latex');

% 按时间分段将背景填充
y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

%%%% 子图1/2 -- 速度X估计误差局部放大图
ax1_1 = axes('Position', [0.2, 0.8, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 0.1));
[~, id_end] = min(abs(tout - 0.3));
axes(ax1_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), eta_obs_err_x(iiFF,id_start:id_end), 'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end

ax1_2 = axes('Position', [0.5, 0.8, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 1.6));
[~, id_end] = min(abs(tout - 2.0));
axes(ax1_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_obs_err_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end

%%%% 子图3/4 -- 速度Y估计误差局部放大图
ax2_1 = axes('Position', [0.2, 0.5, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 0.1));
[~, id_end] = min(abs(tout - 0.3));
axes(ax2_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), eta_obs_err_y(iiFF,id_start:id_end), 'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end

ax2_2 = axes('Position', [0.5, 0.5, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 1.6));
[~, id_end] = min(abs(tout - 2.0));
axes(ax2_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_obs_err_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end

%%%% 子图5/6 -- 速度Z估计误差局部放大图
ax3_1 = axes('Position', [0.2, 0.2, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 0.1));
[~, id_end] = min(abs(tout - 0.3));
axes(ax3_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_obs_err_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end

ax3_2 = axes('Position', [0.5, 0.2, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 1.6));
[~, id_end] = min(abs(tout - 2.0));
axes(ax3_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_obs_err_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end

%% Plot the 3D position trajectories of the UAV formation

k_0_1 = 1:fix( length( T0:dt:T1 ) / 5 ):length( T0:dt:T1 );
k_1_12 = length( T0:dt:(T1+dt) ) + ( 0:fix( length( T1:dt:T12 ) / 5 ):length( T1:dt:T12 ) );
k_12_2 = length( T0:dt:(T12+dt) ) + ( 0:fix( length( T12:dt:T2 ) / 5 ):length( T12:dt:T2 ) );

k_arr = [ k_0_1( 2:2:( length( k_0_1 ) - 1 ) ), fix( k_0_1(end) * ( 4/5 ) ), ...
                    k_1_12( 2:2:( length( k_1_12 ) - 1 ) ), k_12_2( 2:2:( length( k_12_2 ) - 1 ) ) ];

color_arr = { 'r', 'g', 'b', 'c', 'y' };

figure(3)
%%%% 绘制领导者位置曲线
pp0 = plot3(ksi0(1,:), ksi0(2,:), ksi0(3,:), 'color', 'k', ...
                        'linestyle', '-', 'linewidth', 1.0);
hold on;
%%%% 绘制领导者起始位置点
plot3(ksi0(1,1), ksi0(2,1), ksi0(3,1), 'color', 'k', 'marker', 'p', ...
            'markeredgecolor', 'k', 'markerfacecolor', 'k', 'linewidth', 1.0);
hold on;
%%%% 绘制领导者终止位置点
plot3(ksi0(1,end), ksi0(2,end), ksi0(3,end), 'color', 'k', 'marker', 'h', ...
            'markeredgecolor', 'k', 'markerfacecolor', 'k', 'linewidth', 1.0);
hold on;

%%%% 绘制领导者运动方向上的箭头
Nl = size(ksi0,2); % 列数
indices = round(linspace(2, Nl-1, 10)); % 选取5个中间点，避开两端
% 计算切线方向
for i = 1:length(indices)
    idl = indices(i);
    % 使用中心差分估计方向
    if idl > 1 && idl < Nl
        dx = ksi0(1, idl+1) - ksi0(1, idl-1);
        dy = ksi0(2, idl+1) - ksi0(2, idl-1);
        dz = ksi0(3, idl+1) - ksi0(3, idl-1);
    elseif idx == 1
        dx = ksi0(1, 2) - ksi0(1, 1);
        dy = ksi0(2, 2) - ksi0(2, 1);
        dz = ksi0(3, 2) - ksi0(3, 1);
    else % idx == N
        dx = ksi0(1, Nl) - ksi0(1, Nl-1);
        dy = ksi0(2, Nl) - ksi0(2, Nl-1);
        dz = ksi0(3, Nl) - ksi0(3, Nl-1);
    end
    % 归一化并设置箭头长度（例如数据范围的比例）
    vec = [dx; dy; dz];
    vec_len = norm(vec);
    if vec_len > 0
        vec = vec / vec_len; % 单位向量
        % 箭头长度，可以设为数据范围的5%左右
        data_range = max(ksi0(1,:))-min(ksi0(1,:)) + max(ksi0(2,:))-min(ksi0(2,:)) + max(ksi0(3,:))-min(ksi0(3,:));
        arrow_len = data_range * 0.08; % 可根据需要调整
        vec = vec * arrow_len;
        quiver3(ksi0(1,idl), ksi0(2,idl), ksi0(3,idl), vec(1), vec(2), vec(3), ...
                        'color', 'k', 'linewidth', 1.5, 'MaxHeadSize', 2.0);
    end
end
hold on;

for iiFF = 1:5
    %%%% 绘制跟随者位置曲线
    plot3( ksi_fl_x(iiFF, :), ksi_fl_y(iiFF, :), ksi_fl_z(iiFF, :), ...
                'color', color_arr{iiFF}, 'linestyle', '-', 'linewidth', 1.0 );
    hold on;
    %%%% 绘制跟随者位置起点
    plot3( ksi_fl_x(iiFF, 1), ksi_fl_y(iiFF, 1), ksi_fl_z(iiFF, 1), 'color', color_arr{iiFF}, ...
                'marker', 'p', 'linewidth', 1.0, 'markeredgecolor', color_arr{iiFF}, 'markerfacecolor', color_arr{iiFF} );
    hold on;
    %%%% 绘制跟随者位置终点
    plot3( ksi_fl_x(iiFF, end), ksi_fl_y(iiFF, end), ksi_fl_z(iiFF, end), 'color', color_arr{iiFF}, ...
                'marker', 'h', 'linewidth', 1.0, 'markeredgecolor', color_arr{iiFF}, 'markerfacecolor', color_arr{iiFF} );
    hold on;
end

for kkt = k_arr
    %%%% 绘制领导者位置闪照
    pp0 = plot3( ksi0(1, kkt), ksi0(2, kkt), ksi0(3, kkt), ...
                            'color', 'k', 'marker', 'o', 'markersize', 5, ...
                            'markerfacecolor', 'k', 'markeredgecolor', 'k' );
    for iiFF = 1:5
        %%%% 绘制跟随者位置闪照
        ppFk(iiFF) = plot3( ksi_fl_x(iiFF, kkt), ksi_fl_y(iiFF, kkt), ksi_fl_z(iiFF, kkt), ...
                                            'color', color_arr{iiFF}, 'marker', 'o', 'markersize', 5, ...
                                            'markerfacecolor', color_arr{iiFF}, 'markeredgecolor', color_arr{iiFF} );
        hold on;
        %%%% 绘制领导者同跟随者位置队形连线
        plot3( [ ksi0(1, kkt), ksi_fl_x(iiFF, kkt) ], ...
                    [ ksi0(2, kkt), ksi_fl_y(iiFF, kkt) ], ...
                    [ ksi0(3, kkt), ksi_fl_z(iiFF, kkt) ], ...
                    'color', 'm', 'linestyle', '--', 'linewidth', 0.8 );
        hold on;
    end
    %%%% 为跟随者的位置闪照连线展示队形
    for iiFF = 1:4
        plot3( [ ksi_fl_x(iiFF, kkt), ksi_fl_x(iiFF + 1, kkt) ], ...
                    [ ksi_fl_y(iiFF, kkt), ksi_fl_y(iiFF + 1, kkt) ], ...
                    [ ksi_fl_z(iiFF, kkt), ksi_fl_z(iiFF + 1, kkt) ], ...
                    'color', 'm', 'linestyle', '--', 'linewidth', 0.8 );
        hold on;
    end
    plot3( [ ksi_fl_x(NF, kkt), ksi_fl_x(1, kkt) ], ...
                [ ksi_fl_y(NF, kkt), ksi_fl_y(1, kkt) ], ...
                [ ksi_fl_z(NF, kkt), ksi_fl_z(1, kkt) ], ...
                'color', 'm', 'linestyle', '--', 'linewidth', 0.8 );
    hold on;
end

lgd = legend([pp0,ppFk(1),ppFk(2),ppFk(3),ppFk(4),ppFk(5)], ...
                        {'$$i=0$$', '$$i=1$$', '$$i=2$$', ...
                         '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','color','none', ...
                         'box','off','NumColumns',2);
box on;

ax_pos = get(gca, 'Position');
lgd_width = 0.2;
lgd_height = 0.08;
margin_right = -0.17;
margin_bottom = 0.13;
lgd_left = ax_pos(1) + ax_pos(3) - lgd_width - margin_right;
lgd_bottom = ax_pos(2) + margin_bottom;
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_height]);

axis equal
xlabel('$$\it {p}_{i,x} \rm (m)$$','interpreter','latex');
ylabel('$$\it {p}_{i,y} \rm (m)$$','interpreter','latex');
zlabel('$$\it {p}_{i,z} \rm (m)$$','interpreter','latex');

view(-10, 55);

%% Plot the position tracking error curves
linestyle_vec = {'--', '-.', ':'};

figure(4)
subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, ksi_err_fl_x(iiFF,:), 'color', color_vec(id_color,:), ...
                            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
            {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
             'interpreter','latex','location','best','NumColumns',5, ...
             'box','off','color','none');
box on;
xlabel('time (s)');
ylabel('$$\it \tilde {p}_{i,x} \rm (m)$$','interpreter','latex');

% 按时间分段将背景填充
y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, ksi_err_fl_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('time (s)');
ylabel('$$\it \tilde {p}_{i,y} \rm (m)$$','interpreter','latex');

% 按时间分段将背景填充
y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, ksi_err_fl_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('time (s)');
ylabel('$$\it \tilde {p}_{i,z} \rm (m)$$','interpreter','latex');

% 按时间分段将背景填充
y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

%% Plot the velocity tracking error curves

figure(5)
subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, eta_err_fl_x(iiFF,:), 'color', color_vec(id_color,:), ...
                            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
            {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
             'interpreter','latex','location','best','NumColumns',5, ...
             'box','off','color','none');
box on;
set(gca, 'ylim', [-10, 10]);
xlabel('time (s)');
ylabel('$$\it \tilde {v}_{i,x} \rm (m/s)$$','interpreter','latex');

% 按时间分段将背景填充
y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, eta_err_fl_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
set(gca, 'ylim', [-10, 10]);
xlabel('time (s)');
ylabel('$$\it \tilde {v}_{i,y} \rm (m/s)$$','interpreter','latex');

% 按时间分段将背景填充
y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, eta_err_fl_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
set(gca, 'ylim', [-5, 5]);
xlabel('time (s)');
ylabel('$$\it \tilde {v}_{i,z} \rm (m/s)$$','interpreter','latex');

% 按时间分段将背景填充
y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

%% Plot the position estimation results of the differentiator

figure(6)

subplot(531)
pp0 = plot(tout, ksi_fl_x(1,:), 'color', 'r', ...
                    'linestyle', '-', 'linewidth', 1.2);
hold on;
pp1 = plot(tout, ksi_diff_x(1,:), 'color', 'b', ...
                    'linestyle', '--', 'linewidth', 1.2);
hold on;
legend([pp0, pp1], {'$ p_{i} $', '$ \hat{p}_{i} $'}, ...
             'interpreter','latex','location','north','NumColumns',2, ...
             'box','off','color','none');
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{1,x} \rm (m)$$','interpreter','latex');

subplot(532)
plot(tout, ksi_fl_y(1,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_y(1,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{1,y} \rm (m)$$','interpreter','latex');

subplot(533)
plot(tout, ksi_fl_z(1,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_z(1,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{1,z} \rm (m)$$','interpreter','latex');


subplot(534)
plot(tout, ksi_fl_x(2,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_x(2,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{2,x} \rm (m)$$','interpreter','latex');

subplot(535)
plot(tout, ksi_fl_y(2,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_y(2,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{2,y} \rm (m)$$','interpreter','latex');

subplot(536)
plot(tout, ksi_fl_z(2,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_z(2,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{2,z} \rm (m)$$','interpreter','latex');


subplot(537)
plot(tout, ksi_fl_x(3,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_x(3,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{3,x} \rm (m)$$','interpreter','latex');

subplot(538)
plot(tout, ksi_fl_y(3,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_y(3,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{3,y} \rm (m)$$','interpreter','latex');

subplot(539)
plot(tout, ksi_fl_z(3,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_z(3,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{3,z} \rm (m)$$','interpreter','latex');


subplot(5,3,10)
plot(tout, ksi_fl_x(4,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_x(4,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{4,x} \rm (m)$$','interpreter','latex');

subplot(5,3,11)
plot(tout, ksi_fl_y(4,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_y(4,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{4,y} \rm (m)$$','interpreter','latex');

subplot(5,3,12)
plot(tout, ksi_fl_z(4,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_z(4,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{4,z} \rm (m)$$','interpreter','latex');


subplot(5,3,13)
plot(tout, ksi_fl_x(5,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_x(5,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{5,x} \rm (m)$$','interpreter','latex');

subplot(5,3,14)
plot(tout, ksi_fl_y(5,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_y(5,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{5,y} \rm (m)$$','interpreter','latex');

subplot(5,3,15)
plot(tout, ksi_fl_z(5,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, ksi_diff_z(5,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{p}_{5,z} \rm (m)$$','interpreter','latex');


%% Plot the velocity estimation output of the differentiator

figure(7)

subplot(531)
pp0 = plot(tout, eta_fl_x(1,:), 'color', 'r', ...
                    'linestyle', '-', 'linewidth', 1.2);
hold on;
pp1 = plot(tout, eta_diff_x(1,:), 'color', 'b', ...
                    'linestyle', '--', 'linewidth', 1.2);
hold on;
legend([pp0, pp1], {'$ v_{i} $', '$ \hat{v}_{i} $'}, ...
             'interpreter','latex','location','north','NumColumns',2, ...
             'box','off','color','none');
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{1,x} \rm (m)$$','interpreter','latex');

subplot(532)
plot(tout, eta_fl_y(1,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_y(1,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{1,y} \rm (m)$$','interpreter','latex');

subplot(533)
plot(tout, eta_fl_z(1,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_z(1,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{1,z} \rm (m)$$','interpreter','latex');


subplot(534)
plot(tout, eta_fl_x(2,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_x(2,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{2,x} \rm (m)$$','interpreter','latex');

subplot(535)
plot(tout, eta_fl_y(2,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_y(2,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{2,y} \rm (m)$$','interpreter','latex');

subplot(536)
plot(tout, eta_fl_z(2,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_z(2,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{2,z} \rm (m)$$','interpreter','latex');


subplot(537)
plot(tout, eta_fl_x(3,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_x(3,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{3,x} \rm (m)$$','interpreter','latex');

subplot(538)
plot(tout, eta_fl_y(3,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_y(3,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{3,y} \rm (m)$$','interpreter','latex');

subplot(539)
plot(tout, eta_fl_z(3,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_z(3,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{3,z} \rm (m)$$','interpreter','latex');


subplot(5,3,10)
plot(tout, eta_fl_x(4,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_x(4,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{4,x} \rm (m)$$','interpreter','latex');

subplot(5,3,11)
plot(tout, eta_fl_y(4,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_y(4,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{4,y} \rm (m)$$','interpreter','latex');

subplot(5,3,12)
plot(tout, eta_fl_z(4,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_z(4,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{4,z} \rm (m)$$','interpreter','latex');


subplot(5,3,13)
plot(tout, eta_fl_x(5,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_x(5,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{5,x} \rm (m)$$','interpreter','latex');

subplot(5,3,14)
plot(tout, eta_fl_y(5,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_y(5,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{5,y} \rm (m)$$','interpreter','latex');

subplot(5,3,15)
plot(tout, eta_fl_z(5,:), 'color', 'r', ...
            'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, eta_diff_z(5,:), 'color', 'b', ...
            'linestyle', '--', 'linewidth', 1.2);
hold on;
box on;
xlabel('time (s)');
ylabel('$$\it \hat{v}_{5,z} \rm (m)$$','interpreter','latex');

%% Plot the true and estimated disturbance values
kk_plot_disturb = 1;
kk_gain = 10;

figure(8)
subplot(311)
ppk(1) = plot(tout(1,:), dp_x(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout(1,:), dp_hat_x(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it \hat {d}_{i,x}^{v} \rm (m/s)$$','interpreter','latex');
legend([ppk(1),ppk(2)], ...
            {'$$\it {d}_{i}^{v}$$', '$$\it \hat {d}_{i}^{v}$$'}, ...
             'interpreter','latex','location','best', ...
             'NumColumns',2,'box','off','color','none');
set(gca, 'ylim', [-0.5, 0.5] .* kk_gain);
 
subplot(312)
ppk(1) = plot(tout(1,:), dp_y(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout(1,:), dp_hat_y(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it \hat {d}_{i,y}^{v} \rm (m/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.2, 0.4] .* kk_gain);

subplot(313)
ppk(1) = plot(tout(1,:), dp_z(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout(1,:), dp_hat_z(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it \hat {d}_{i,z}^{v} \rm (m/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.5, 0.5] .* kk_gain);

%% 
