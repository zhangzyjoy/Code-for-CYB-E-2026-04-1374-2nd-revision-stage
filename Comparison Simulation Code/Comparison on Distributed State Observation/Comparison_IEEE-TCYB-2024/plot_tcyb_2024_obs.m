%% Plot results for IEEE TCYB (2024) paper:
%% <Resilient Neuroadaptive Distributed Fixed-Time
%% Attitude Coordination Control for Multiple Spacecraft>

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
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','box','off','color','none','NumColumns',5);
box on;
xlabel('time (s)');
ylabel('$$\it \tilde{p}_{i,x}^{d} \rm (m)$$','interpreter','latex');

%%%% 第一个子图上方显示图例
ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);

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
ax1_1 = axes('Position', [0.2, 0.78, 0.15, 0.1]);
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.45, 0.78, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 10.5));
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.78, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 20.5));
axes(ax1_3)
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.45, 0.5, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 10.5));
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.5, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 20.5));
axes(ax2_3)
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图5 -- 位置Z估计误差局部放大图
ax3_1 = axes('Position', [0.2, 0.24, 0.15, 0.08]);
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
            ksi_obs_err_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_2 = axes('Position', [0.45, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 10.5));
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
            ksi_obs_err_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 20.5));
axes(ax3_3)
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
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
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','box','off','color','none','NumColumns',5);
box on;
xlabel('time (s)');
ylabel('$$\it \tilde{v}_{i,x}^{d} \rm (m/s)$$','interpreter','latex');

%%%% 第一个子图上方显示图例
ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);

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
ax1_1 = axes('Position', [0.2, 0.87, 0.15, 0.05]);
[~, id_start] = min(abs(tout - 0.05));
[~, id_end] = min(abs(tout - 0.2));
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.45, 0.87, 0.15, 0.05]);
[~, id_start] = min(abs(tout - 10.1));
[~, id_end] = min(abs(tout - 10.4));
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.87, 0.15, 0.05]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 20.2));
axes(ax1_3)
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图3/4 -- 速度Y估计误差局部放大图
ax2_1 = axes('Position', [0.2, 0.54, 0.15, 0.08]);
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.45, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.1));
[~, id_end] = min(abs(tout - 10.4));
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 20.6));
axes(ax2_3)
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图5/6 -- 速度Z估计误差局部放大图
ax3_1 = axes('Position', [0.2, 0.18, 0.15, 0.1]);
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_2 = axes('Position', [0.45, 0.18, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 10.1));
[~, id_end] = min(abs(tout - 10.4));
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.18, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 20.6));
axes(ax3_3)
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
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end


%% 
