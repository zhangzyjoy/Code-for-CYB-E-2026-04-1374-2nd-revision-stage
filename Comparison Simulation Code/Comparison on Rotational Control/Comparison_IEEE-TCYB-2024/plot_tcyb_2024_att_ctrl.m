%% Plot results for IEEE-TCYB (2024) paper:
%% <Adaptive Formation Tracking Control of Multiple
%% Vertical Takeoff and Landing UAVs With Bearing-Only Measurements>

%% Run <init_tcyb_2024_att_ctrl.m> to initialize
%% Run <sim_tcyb_2024_att_ctrl.slx> to load the simulation model
%% Run <plot_tcyb_2024_att_ctrl.m> to plot the attitude control and disturbance observation results

clc;
close all;

linestyle_vec = {'--', '-.', ':'};

color_vec = [252, 170, 103; ...
                        189, 30, 30; ...
                        124, 187, 0; ...
                        54, 195, 201; ...
                        0, 70, 222; ...
                        0, 0, 0] ./ 255;

%%  Extract simulation data
NF = 5;
tLen = length(tout);

%%%% Rodrigues parameter error
sgei_x = zeros(NF,tLen);
sgei_y = zeros(NF,tLen);
sgei_z = zeros(NF,tLen);

%%%% Angular velocity error
wei_x = zeros(NF,tLen);
wei_y = zeros(NF,tLen);
wei_z = zeros(NF,tLen);

%%%% Attitude rotation error in the LogSO(3) space
V_Phi_Rei_x = zeros(NF,tLen);
V_Phi_Rei_y = zeros(NF,tLen);
V_Phi_Rei_z = zeros(NF,tLen);

%%%% Angular velocity
wi_x = zeros(NF,tLen);
wi_y = zeros(NF,tLen);
wi_z = zeros(NF,tLen);

%%%% Adaptive variable
zetai_x = zeros(NF,tLen);
zetai_y = zeros(NF,tLen);
zetai_z = zeros(NF,tLen);

%%%% Adaptive variable
thetai_hat_1 = zeros(NF,tLen);
thetai_hat_2 = zeros(NF,tLen);
thetai_hat_3 = zeros(NF,tLen);
thetai_hat_4 = zeros(NF,tLen);
thetai_hat_5 = zeros(NF,tLen);
thetai_hat_6 = zeros(NF,tLen);

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
    thetai = thetai';
end

%%%% Extract simulation data
for tt = 1:tLen
    for iiFF = 1:NF
        sgei_x(iiFF,tt) = sgei(1,iiFF,tt);
        sgei_y(iiFF,tt) = sgei(2,iiFF,tt);
        sgei_z(iiFF,tt) = sgei(3,iiFF,tt);
        wei_x(iiFF,tt) = wei(1,iiFF,tt);
        wei_y(iiFF,tt) = wei(2,iiFF,tt);
        wei_z(iiFF,tt) = wei(3,iiFF,tt);
        V_Phi_Rei_x(iiFF,tt) = V_Phi_Rei(1,iiFF,tt);
        V_Phi_Rei_y(iiFF,tt) = V_Phi_Rei(2,iiFF,tt);
        V_Phi_Rei_z(iiFF,tt) = V_Phi_Rei(3,iiFF,tt);
        wi_x(iiFF,tt) = wi(1,iiFF,tt);
        wi_y(iiFF,tt) = wi(2,iiFF,tt);
        wi_z(iiFF,tt) = wi(3,iiFF,tt);
        zetai_x(iiFF,tt) = zetai(1,iiFF,tt);
        zetai_y(iiFF,tt) = zetai(2,iiFF,tt);
        zetai_z(iiFF,tt) = zetai(3,iiFF,tt);
        thetai_hat_1(iiFF,tt) = thetai_hat(1,iiFF,tt);
        thetai_hat_2(iiFF,tt) = thetai_hat(2,iiFF,tt);
        thetai_hat_3(iiFF,tt) = thetai_hat(3,iiFF,tt);
        thetai_hat_4(iiFF,tt) = thetai_hat(4,iiFF,tt);
        thetai_hat_5(iiFF,tt) = thetai_hat(5,iiFF,tt);
        thetai_hat_6(iiFF,tt) = thetai_hat(6,iiFF,tt);
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
figure(1)
clear ppk pp0

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
legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
            {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
             'interpreter','latex','location','best','NumColumns',5, ...
             'box','off','color','none');
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\omega}_{i}^{e}(x) \rm \ (rad/s)$$','interpreter','latex');

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
ylabel('$$\it {\omega}_{i}^{e}(y) \rm \ (rad/s)$$','interpreter','latex');

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
ylabel('$$\it {\omega}_{i}^{e}(z) \rm \ (rad/s)$$','interpreter','latex');

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

%% Plot the attitude rotation error in the LogSO(3) space
figure(2)
clear ppk pp0

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
legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
            {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
             'interpreter','latex','location','best','NumColumns',5, ...
             'box','off','color','none');
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\Psi}[(R(Q_{i}^{e}))]_{\vee}(x) \rm \ (rad/s)$$','interpreter','latex');

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
ylabel('$$\it {\Psi}[(R(Q_{i}^{e}))]_{\vee}(y) \rm \ (rad/s)$$','interpreter','latex');

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
ylabel('$$\it {\Psi}[(R(Q_{i}^{e}))]_{\vee}(z) \rm \ (rad/s)$$','interpreter','latex');

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

%% Plot the Rodrigues parameter tracking error
figure(3)
clear ppk pp0

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
    ppk(iiFF) = plot(tout, sgei_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
            {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
             'interpreter','latex','location','best','NumColumns',5, ...
             'box','off','color','none');
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\sigma}_{i}^{e}(x) \rm$$','interpreter','latex');

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
    ppk(iiFF) = plot(tout, sgei_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\sigma}_{i}^{e}(y) \rm$$','interpreter','latex');

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
    ppk(iiFF) = plot(tout, sgei_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\sigma}_{i}^{e}(z) \rm$$','interpreter','latex');

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

%% Plot the adaptive parameters
figure(4)
clear ppk pp0

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
    ppk(iiFF) = plot(tout, zetai_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
            {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
             'interpreter','latex','location','best','NumColumns',5, ...
             'box','off','color','none');
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\zeta}_{i}(x) \rm \ (rad/s)$$','interpreter','latex');

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
    ppk(iiFF) = plot(tout, zetai_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\zeta}_{i}(y) \rm \ (rad/s)$$','interpreter','latex');

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
    ppk(iiFF) = plot(tout, zetai_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\zeta}_{i}(z) \rm \ (rad/s)$$','interpreter','latex');

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

%% Plot the adaptive parameters
figure(5)

subplot(321)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) =plot(tout, thetai_hat_1(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
pp0 = plot(tout, thetai(1,:), 'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold off;

fig = gcf;
ax_1 = axes('Parent', fig, 'Units', 'normalized', 'Position', get(gca, 'Position'), ...
                               'Visible', 'on', 'Color', 'none', 'XTick', [], 'YTick', [], ...
                               'XColor', 'none', 'YColor', 'none', 'Box', 'off');
ax_1.Visible = 'off';

lgd1 = legend(ax_1, pp0, {'$J_{i}^{1,1}$'}, 'interpreter','latex', ...
                            'location','northwest', 'box','off','color','none');
lgd1.AutoUpdate = 'off';


ax_2 = axes('Parent', fig, 'Units', 'normalized', 'Position', get(gca, 'Position'), ...
                               'Visible', 'on', 'Color', 'none', 'XTick', [], 'YTick', [], ...
                               'XColor', 'none', 'YColor', 'none', 'Box', 'off');
ax_2.Visible = 'off';

lgd2 = legend(ax_2, [ppk(1), ppk(2), ppk(3), ppk(4), ppk(5)], ...
                          {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                          'interpreter','latex', 'NumColumns', 5, 'location','northeast', ...
                          'box','off','color','none');



box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat{\theta}_{i}(1) \rm \ (rad/s)$$','interpreter','latex');


subplot(322)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout, thetai_hat_2(iiFF,:), 'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
pp0 = plot(tout, thetai(2,:), 'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
legend(pp0, {'$J_{i}^{2,2}$'}, ...
             'interpreter','latex','location','best', ...
             'NumColumns',1,'box','off','color','none');
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat{\theta}_{i}(2) \rm \ (rad/s)$$','interpreter','latex');

subplot(323)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout, thetai_hat_3(iiFF,:), 'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
pp0 = plot(tout, thetai(3,:), 'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
legend(pp0, {'$J_{i}^{3,3}$'}, ...
             'interpreter','latex','location','best', ...
             'NumColumns',1,'box','off','color','none');
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat{\theta}_{i}(3) \rm \ (rad/s)$$','interpreter','latex');

subplot(324)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout, thetai_hat_4(iiFF,:), 'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
pp0 = plot(tout, thetai(4,:), 'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
legend(pp0, {'$J_{i}^{2,3}$'}, ...
             'interpreter','latex','location','best', ...
             'NumColumns',1,'box','off','color','none');
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat{\theta}_{i}(4) \rm \ (rad/s)$$','interpreter','latex');


subplot(325)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout, thetai_hat_5(iiFF,:), 'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
pp0 = plot(tout, thetai(5,:), 'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
legend(pp0, {'$J_{i}^{1,3}$'}, ...
             'interpreter','latex','location','best', ...
             'NumColumns',1,'box','off','color','none');
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat{\theta}_{i}(5) \rm \ (rad/s)$$','interpreter','latex');

subplot(326)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout, thetai_hat_6(iiFF,:), 'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
pp0 = plot(tout, thetai(6,:), 'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
legend(pp0, {'$J_{i}^{1,2}$'}, ...
             'interpreter','latex','location','best', ...
             'NumColumns',1,'box','off','color','none');
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat{\theta}_{i}(6) \rm \ (rad/s)$$','interpreter','latex');


%% Plot the angular velocity tracking performance: angular velocity and auxiliary angular velocity

kk_plot_disturb = 1;
kdgain = 10;

figure(6)
clear ppk pp0

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
             'interpreter','latex','location','best','box','off','color','none');
% set(gca, 'ylim', [-0.6 * kdgain, 0.2 * kdgain]);

subplot(312)
ppk(1) = plot(tout, wi_y(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_w_y(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\sigma}_{i,y}^{\varpi}  \rm (rad/s)$$','interpreter','latex');
% set(gca, 'ylim', [-0.2 * kdgain, 0.6 * kdgain]);

subplot(313)
ppk(1) = plot(tout, wi_z(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_w_z(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\sigma}_{i,z}^{\varpi}  \rm (rad/s)$$','interpreter','latex');
% set(gca, 'ylim', [-0.5 * kdgain, 0.2 * kdgain]);


%% Comparison between the angular velocity tracking error and its observation

figure(7)
clear ppk pp0

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
             'interpreter','latex','location','best','box','off','color','none');
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
figure(8)
clear ppk pp0

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
             'interpreter','latex','location','best','box','off','color','none');
set(gca, 'ylim', [-0.6 * kdgain, 0.6 * kdgain]);

subplot(312)
ppk(1) = plot(tout, dwi_y(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, dwi_hat_y(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat {d}_{i,y}^{\varpi} \rm (rad/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.8 * kdgain, 0.6 * kdgain]);

subplot(313)
ppk(1) = plot(tout, dwi_z(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, dwi_hat_z(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat {d}_{i,z}^{\varpi} \rm (rad/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.7 * kdgain, 0.7 * kdgain]);

%% 
