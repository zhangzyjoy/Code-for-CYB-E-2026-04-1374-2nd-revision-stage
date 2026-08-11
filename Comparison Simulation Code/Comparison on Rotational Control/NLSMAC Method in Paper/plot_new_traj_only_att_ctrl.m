%% Plot the results in the submitted manuscript :
%% Nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC)
%% Parameter settings for fixed-time disturbance observer (FxTDO) in Rotational channel

%% Run <init_new_traj_only_att_ctrl.m> to initialize the parameters and system states
%% Run <sim_new_traj_only_att_ctrl.slx> to execute the attitude control simulation
%% Run <plot_new_traj_only_att_ctrl.m> to plot the simulation results

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

%%%% Quaternion error
%%%% Qei = [Qei_th, Qei_qx, Qei_qy, Qei_qz]'
Qei_th = zeros(NF,tLen);
Qei_qx = zeros(NF,tLen);
Qei_qy = zeros(NF,tLen);
Qei_qz = zeros(NF,tLen);

%%%% Angular velocity error
wei_x = zeros(NF,tLen);
wei_y = zeros(NF,tLen);
wei_z = zeros(NF,tLen);

%%%% Angular velocity
wi_x = zeros(NF,tLen);
wi_y = zeros(NF,tLen);
wi_z = zeros(NF,tLen);

%%%% Attitude rotation error in the LogSO(3) space
V_Phi_Rei_x = zeros(NF,tLen);
V_Phi_Rei_y = zeros(NF,tLen);
V_Phi_Rei_z = zeros(NF,tLen);

%%%% Auxiliary attitude error
Phi_psi_e_x = zeros(NF,tLen);
Phi_psi_e_y = zeros(NF,tLen);
Phi_psi_e_z = zeros(NF,tLen);

%%%% Time derivative of the auxiliary attitude error
Phi_bar_psi_e_x = zeros(NF,tLen);
Phi_bar_psi_e_y = zeros(NF,tLen);
Phi_bar_psi_e_z = zeros(NF,tLen);

%%%% Auxiliary sliding mode surface
S_bar_x = zeros(NF,tLen);
S_bar_y = zeros(NF,tLen);
S_bar_z = zeros(NF,tLen);

%%%% Sliding mode surface
Si_x = zeros(NF,tLen);
Si_y = zeros(NF,tLen);
Si_z = zeros(NF,tLen);

%%%% Auxiliary angular velocity
sigma_w_x = zeros(NF,tLen);
sigma_w_y = zeros(NF,tLen);
sigma_w_z = zeros(NF,tLen);

%%%% Angular velocity tracking error
sigma_bar_w_x = zeros(NF,tLen);
sigma_bar_w_y = zeros(NF,tLen);
sigma_bar_w_z = zeros(NF,tLen);

%%%% Estimated angular velocity tracking error
sigma_bar_w_hat_x = zeros(NF,tLen);
sigma_bar_w_hat_y = zeros(NF,tLen);
sigma_bar_w_hat_z = zeros(NF,tLen);

%%%% Custom disturbances defined in the simulation
dwi_x = zeros(NF,tLen);
dwi_y = zeros(NF,tLen);
dwi_z = zeros(NF,tLen);

%%%% Estimated disturbance values
dwi_hat_x = zeros(NF,tLen);
dwi_hat_y = zeros(NF,tLen);
dwi_hat_z = zeros(NF,tLen);

%%%% Disturbance estimation errors
dwi_tilt_x = zeros(NF,tLen);
dwi_tilt_y = zeros(NF,tLen);
dwi_tilt_z = zeros(NF,tLen);

%%%% Correct data format
if size(tout,2) == 1
    tout = tout';
end

%%%% Extract simulation data
for tt = 1:tLen
    for iiFF = 1:NF
        Qei_th(iiFF,tt) = Qei(1,iiFF,tt);
        Qei_qx(iiFF,tt) = Qei(2,iiFF,tt);
        Qei_qy(iiFF,tt) = Qei(3,iiFF,tt);
        Qei_qz(iiFF,tt) = Qei(4,iiFF,tt);
        wei_x(iiFF,tt) = wei(1,iiFF,tt);
        wei_y(iiFF,tt) = wei(2,iiFF,tt);
        wei_z(iiFF,tt) = wei(3,iiFF,tt);
        wi_x(iiFF,tt) = wi(1,iiFF,tt);
        wi_y(iiFF,tt) = wi(2,iiFF,tt);
        wi_z(iiFF,tt) = wi(3,iiFF,tt);
        V_Phi_Rei_x(iiFF,tt) = V_Phi_Rei(1,iiFF,tt);
        V_Phi_Rei_y(iiFF,tt) = V_Phi_Rei(2,iiFF,tt);
        V_Phi_Rei_z(iiFF,tt) = V_Phi_Rei(3,iiFF,tt);
        Phi_psi_e_x(iiFF,tt) = Phi_psi_e(1,iiFF,tt);
        Phi_psi_e_y(iiFF,tt) = Phi_psi_e(2,iiFF,tt);
        Phi_psi_e_z(iiFF,tt) = Phi_psi_e(3,iiFF,tt);
        Phi_bar_psi_e_x(iiFF,tt) = Phi_bar_psi_e(1,iiFF,tt);
        Phi_bar_psi_e_y(iiFF,tt) = Phi_bar_psi_e(2,iiFF,tt);
        Phi_bar_psi_e_z(iiFF,tt) = Phi_bar_psi_e(3,iiFF,tt);
        S_bar_x(iiFF,tt) = S_bar(1,iiFF,tt);
        S_bar_y(iiFF,tt) = S_bar(2,iiFF,tt);
        S_bar_z(iiFF,tt) = S_bar(3,iiFF,tt);
        Si_x(iiFF,tt) = Si(1,iiFF,tt);
        Si_y(iiFF,tt) = Si(2,iiFF,tt);
        Si_z(iiFF,tt) = Si(3,iiFF,tt);
        sigma_w_x(iiFF,tt) = sigma_w(1,iiFF,tt);
        sigma_w_y(iiFF,tt) = sigma_w(2,iiFF,tt);
        sigma_w_z(iiFF,tt) = sigma_w(3,iiFF,tt);
        sigma_bar_w_x(iiFF,tt) = sigma_bar_w(1,iiFF,tt);
        sigma_bar_w_y(iiFF,tt) = sigma_bar_w(2,iiFF,tt);
        sigma_bar_w_z(iiFF,tt) = sigma_bar_w(3,iiFF,tt);
        sigma_bar_w_hat_x(iiFF,tt) = sigma_bar_w_hat(1,iiFF,tt);
        sigma_bar_w_hat_y(iiFF,tt) = sigma_bar_w_hat(2,iiFF,tt);
        sigma_bar_w_hat_z(iiFF,tt) = sigma_bar_w_hat(3,iiFF,tt);
        dwi_x(iiFF,tt) = dwi(1,iiFF,tt);
        dwi_y(iiFF,tt) = dwi(2,iiFF,tt);
        dwi_z(iiFF,tt) = dwi(3,iiFF,tt);
        dwi_hat_x(iiFF,tt) = dwi_hat(1,iiFF,tt);
        dwi_hat_y(iiFF,tt) = dwi_hat(2,iiFF,tt);
        dwi_hat_z(iiFF,tt) = dwi_hat(3,iiFF,tt);
        dwi_tilt_x(iiFF,tt) = dwi_tilt(1,iiFF,tt);
        dwi_tilt_y(iiFF,tt) = dwi_tilt(2,iiFF,tt);
        dwi_tilt_z(iiFF,tt) = dwi_tilt(3,iiFF,tt);
    end
end

t0 = 0;
t1 = 10;
t12 = 20;
t2 = 30;
del_t = 0.1;

%% Plot the angular velocity tracking error
figure(9)

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
    ppk(iiFF) = plot(tout, wei_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','box','off', 'color','none','NumColumns',5);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\omega}_{i,x}^{e} \rm \ (rad/s)$$','interpreter','latex');

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
    ppk(iiFF) = plot(tout, wei_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\omega}_{i,y}^{e} \rm \ (rad/s)$$','interpreter','latex');

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
    ppk(iiFF) = plot(tout, wei_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\omega}_{i,z}^{e} \rm \ (rad/s)$$','interpreter','latex');

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

%%%% 子图1
ax1_1 = axes('Position', [0.23, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 2.2));
[~, id_end] = min(abs(tout - 2.7));
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
    plot(tout(1,id_start:id_end), ...
            wei_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.46, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 11.4));
[~, id_end] = min(abs(tout - 12.0));
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
            wei_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
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
            wei_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图2
ax2_1 = axes('Position', [0.23, 0.47, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.4));
[~, id_end] = min(abs(tout - 2.2));
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
    plot(tout(1,id_start:id_end), ...
            wei_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.46, 0.47, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.4));
[~, id_end] = min(abs(tout - 11.2));
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
            wei_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.47, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
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
            wei_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图3
ax3_1 = axes('Position', [0.23, 0.23, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 2.0));
[~, id_end] = min(abs(tout - 4.4));
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
            wei_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax3_2 = axes('Position', [0.46, 0.23, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.2));
[~, id_end] = min(abs(tout - 11.0));
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
            wei_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.23, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 22.0));
[~, id_end] = min(abs(tout - 23.0));
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
            wei_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%% Plot the attitude rotation error in the LogSO(3) space
figure(10)

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
    ppk(iiFF) = plot(tout, V_Phi_Rei_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','box','off','color','none','NumColumns',5);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\psi}_{i,x}^{e} \rm \ (rad/s)$$','interpreter','latex');

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
    ppk(iiFF) = plot(tout, V_Phi_Rei_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\psi}_{i,y}^{e} \rm \ (rad/s)$$','interpreter','latex');

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
    ppk(iiFF) = plot(tout, V_Phi_Rei_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\psi}_{i,z}^{e} \rm \ (rad/s)$$','interpreter','latex');

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

%%%% 子图1
ax1_1 = axes('Position', [0.23, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.5));
[~, id_end] = min(abs(tout - 2.5));
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
    plot(tout(1,id_start:id_end), ...
            V_Phi_Rei_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.46, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 11.5));
[~, id_end] = min(abs(tout - 14.0));
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
            V_Phi_Rei_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
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
            V_Phi_Rei_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图2
ax2_1 = axes('Position', [0.23, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.4));
[~, id_end] = min(abs(tout - 3.0));
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
    plot(tout(1,id_start:id_end), ...
            V_Phi_Rei_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.46, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.4));
[~, id_end] = min(abs(tout - 11.2));
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
            V_Phi_Rei_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.5));
[~, id_end] = min(abs(tout - 22.0));
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
            V_Phi_Rei_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图3
ax3_1 = axes('Position', [0.23, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 2.0));
[~, id_end] = min(abs(tout - 5.0));
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
            V_Phi_Rei_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_2 = axes('Position', [0.46, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.2));
[~, id_end] = min(abs(tout - 11.0));
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
            V_Phi_Rei_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.5));
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
            V_Phi_Rei_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%% Plot the auxiliary attitude error
figure(11)

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
    ppk(iiFF) = plot(tout, Phi_psi_e_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','color','none','box','off','NumColumns',5);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\Phi}_{x}({\psi}_{i,x}^{e}) \rm \ (rad/s)$$','interpreter','latex');

%%%% 第一个子图上方显示图例
ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);

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
    ppk(iiFF) = plot(tout, Phi_psi_e_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\Phi}_{y}({\psi}_{i,y}^{e}) \rm \ (rad/s)$$','interpreter','latex');

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
    ppk(iiFF) = plot(tout, Phi_psi_e_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\Phi}_{z}({\psi}_{i,z}^{e}) \rm \ (rad/s)$$','interpreter','latex');

%%%% 子图1
ax1_1 = axes('Position', [0.23, 0.77, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.0));
[~, id_end] = min(abs(tout - 2.5));
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
    plot(tout(1,id_start:id_end), ...
            Phi_psi_e_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.46, 0.77, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 11.4));
[~, id_end] = min(abs(tout - 12.4));
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
            Phi_psi_e_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.77, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
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
            Phi_psi_e_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图2
ax2_1 = axes('Position', [0.23, 0.53, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.0));
[~, id_end] = min(abs(tout - 2.5));
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
    plot(tout(1,id_start:id_end), ...
            Phi_psi_e_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.46, 0.53, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.2));
[~, id_end] = min(abs(tout - 11.0));
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
            Phi_psi_e_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.53, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
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
            Phi_psi_e_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图3
ax3_1 = axes('Position', [0.23, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 2.0));
[~, id_end] = min(abs(tout - 4.4));
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
            Phi_psi_e_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax3_2 = axes('Position', [0.46, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 11.5));
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
            Phi_psi_e_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end
set(gca, 'ylim', [-0.02, 0.02]);

ax3_3 = axes('Position', [0.7, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.0));
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
            Phi_psi_e_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%% Plot the time derivative of the auxiliary attitude error
figure(12)

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
    ppk(iiFF) = plot(tout, Phi_bar_psi_e_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','color','none','box','off','NumColumns',5);
box on;
set(gca, 'ylim', [-1.0, 1.0]);
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \overline{\Phi}_{x}({\psi}_{i,x}^{e}) \rm \ (rad/s)$$','interpreter','latex');

%%%% 第一个子图上方显示图例
ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);


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
    ppk(iiFF) = plot(tout, Phi_bar_psi_e_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
set(gca, 'ylim', [-1.0, 1.0]);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \overline{\Phi}_{y}({\psi}_{i,y}^{e}) \rm \ (rad/s)$$','interpreter','latex');

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
    ppk(iiFF) = plot(tout, Phi_bar_psi_e_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
set(gca, 'ylim', [-1.0, 1.0]);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \overline{\Phi}_{z}({\psi}_{i,z}^{e}) \rm \ (rad/s)$$','interpreter','latex');


%%%% 子图1
ax1_1 = axes('Position', [0.23, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.0));
[~, id_end] = min(abs(tout - 2.5));
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
    plot(tout(1,id_start:id_end), ...
            Phi_bar_psi_e_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.48, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 11.4));
[~, id_end] = min(abs(tout - 11.8));
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
            Phi_bar_psi_e_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
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
            Phi_bar_psi_e_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图2
ax2_1 = axes('Position', [0.23, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.0));
[~, id_end] = min(abs(tout - 2.5));
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
    plot(tout(1,id_start:id_end), ...
            Phi_bar_psi_e_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end
set(gca,'ylim',[-1.0, 2.0]);

ax2_2 = axes('Position', [0.48, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 11.0));
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
            Phi_bar_psi_e_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end
set(gca,'ylim',[-0.4, 0.4]);

ax2_3 = axes('Position', [0.7, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
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
            Phi_bar_psi_e_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图3
ax3_1 = axes('Position', [0.23, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 2.0));
[~, id_end] = min(abs(tout - 3.4));
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
            Phi_bar_psi_e_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax3_2 = axes('Position', [0.48, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.4));
[~, id_end] = min(abs(tout - 12.0));
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
            Phi_bar_psi_e_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end
set(gca, 'ylim', [-0.02, 0.02]);

ax3_3 = axes('Position', [0.7, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.0));
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
            Phi_bar_psi_e_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end


%% Plot the auxiliary sliding mode surface
figure(13)

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
    ppk(iiFF) = plot(tout, S_bar_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','box','off','color','none','NumColumns',5);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \overline{S}_{i,x} \rm \ (rad/s)$$','interpreter','latex');

%%%% 第一个子图上方显示图例
ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);

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
    ppk(iiFF) = plot(tout, S_bar_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \overline{S}_{i,y} \rm \ (rad/s)$$','interpreter','latex');

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
    ppk(iiFF) = plot(tout, S_bar_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \overline{S}_{i,z} \rm \ (rad/s)$$','interpreter','latex');


%%%% 子图1
ax1_1 = axes('Position', [0.23, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.5));
[~, id_end] = min(abs(tout - 3.0));
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
    plot(tout(1,id_start:id_end), ...
            S_bar_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.48, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 11.0));
[~, id_end] = min(abs(tout - 12.5));
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
            S_bar_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
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
            S_bar_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图2
ax2_1 = axes('Position', [0.23, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 0.6));
[~, id_end] = min(abs(tout - 2.6));
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
    plot(tout(1,id_start:id_end), ...
            S_bar_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.48, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 11.0));
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
            S_bar_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.0));
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
            S_bar_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图3
ax3_1 = axes('Position', [0.23, 0.17, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.6));
[~, id_end] = min(abs(tout - 5.0));
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
            S_bar_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax3_2 = axes('Position', [0.48, 0.17, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 12.0));
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
            S_bar_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.17, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.0));
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
            S_bar_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end


%% Plot the sliding mode surface
figure(14)

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
    ppk(iiFF) = plot(tout, Si_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','box','off','color','none','NumColumns',5);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it S_{i}(x) \rm \ (rad/s)$$','interpreter','latex');

%%%% 第一个子图上方显示图例
ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);


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
    ppk(iiFF) = plot(tout, Si_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it S_{i}(y) \rm \ (rad/s)$$','interpreter','latex');

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
    ppk(iiFF) = plot(tout, Si_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it S_{i}(z) \rm \ (rad/s)$$','interpreter','latex');


%%%% 子图1
ax1_1 = axes('Position', [0.23, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.5));
[~, id_end] = min(abs(tout - 3.0));
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
    plot(tout(1,id_start:id_end), ...
            Si_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.48, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 11.0));
[~, id_end] = min(abs(tout - 12.5));
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
            Si_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
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
            Si_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图2
ax2_1 = axes('Position', [0.23, 0.515, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 0.6));
[~, id_end] = min(abs(tout - 2.6));
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
    plot(tout(1,id_start:id_end), ...
            Si_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.48, 0.515, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 12.0));
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
            Si_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.515, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.0));
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
            Si_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图3
ax3_1 = axes('Position', [0.23, 0.17, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.6));
[~, id_end] = min(abs(tout - 3.0));
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
            Si_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax3_2 = axes('Position', [0.48, 0.17, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 11.5));
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
            Si_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.17, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.0));
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
            Si_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end


%% Plot the angular velocity tracking performance: angular velocity and auxiliary angular velocity
kk_plot_disturb = 1;
kdgain = 10;

figure(15)

subplot(311)
ppk(1) = plot(tout, wi_x(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_w_x(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\sigma}_{i,x}^{\varpi}  \rm (rad/s)$$','interpreter','latex');
legend([ppk(1),ppk(2)], ...
            {'$$\it {\varpi}_{i} $$', '$$\it {\sigma}_{i}^{\varpi}$$'}, ...
             'interpreter','latex','location','northeast','color','none','box','off','NumColumns',2);

subplot(312)
ppk(1) = plot(tout, wi_y(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_w_y(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\sigma}_{i,y}^{\varpi}  \rm (rad/s)$$','interpreter','latex');

subplot(313)
ppk(1) = plot(tout, wi_z(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_w_z(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\sigma}_{i,z}^{\varpi}  \rm (rad/s)$$','interpreter','latex');

%% Comparison between the angular velocity tracking error and its observation

figure(16)

subplot(311)
ppk(1) = plot(tout, sigma_bar_w_x(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_bar_w_hat_x(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat { \overline {\sigma}}_{i,x}^{\varpi} \rm (rad/s)$$','interpreter','latex');
legend([ppk(1),ppk(2)], ...
            {'$$\it \overline {\sigma}_{i}^{\varpi}$$', '$$\it \hat { \overline {\sigma}}_{i}^{\varpi}$$'}, ...
             'interpreter','latex','location','southeast','NumColumns',2,'color','none','box','off');
set(gca, 'ylim', [-0.6, 0.3]);

subplot(312)
ppk(1) = plot(tout, sigma_bar_w_y(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_bar_w_hat_y(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat { \overline {\sigma}}_{i,y}^{\varpi} \rm (rad/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.4, 0.6]);

subplot(313)
ppk(1) = plot(tout, sigma_bar_w_z(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_bar_w_hat_z(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat { \overline {\sigma}}_{i,z}^{\varpi} \rm (m/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.5, 0.3]);

%% Comparison between the external disturbance and its observation

figure(17)

subplot(311)
ppk(1) = plot(tout, dwi_x(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, dwi_hat_x(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat {d}_{i,x}^{\varpi} \rm (rad/s)$$','interpreter','latex');
legend([ppk(1),ppk(2)], ...
            {'$$\it {d}_{i}^{\varpi}$$', '$$\it \hat {d}_{i}^{\varpi}$$'}, ...
             'interpreter','latex','location','southeast','NumColumns',2,'color','none','box','off');
set(gca, 'ylim', [-1.0 * kdgain, 1.0 * kdgain]);

subplot(312)
ppk(1) = plot(tout, dwi_y(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, dwi_hat_y(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat {d}_{i,y}^{\varpi} \rm (rad/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.8 * kdgain, 0.8 * kdgain]);

subplot(313)
ppk(1) = plot(tout, dwi_z(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, dwi_hat_z(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat {d}_{i,z}^{\varpi} \rm (rad/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.8 * kdgain, 1.0 * kdgain]);


%% 
