%% Attitude control: comparison of external disturbance observers
%% The compared methods are listed as follows:

%% Method 1: Proposed method
%% The simulation model for Method 1 is <sim_dist_att_obs_paper.slx>

%% Method 2: IEEE TCYB (2024)
%% <Disturbance Rejection Event-Triggered Robust Model Predictive Control
%% for Tracking of Constrained Uncertain Robotic Manipulators>
%% The simulation model for Method 2 is <sim_att_dist_obs_tcyb.slx>

%% Method 3: IEEE TIE (2020)
%% <Composite Disturbance Rejection Attitude Control
%% for Quadrotors With Unknown Disturbances>
%% The simulation model for Method 3 is <sim_att_dist_obs_tie_2020.slx>

%% Method 4: IEEE TIE (2025)
%% <Fixed-Time Angle Tracking Control for Multi-DOF Manipulators
%% Driven by Pneumatic Artificial Muscles>
%% The simulation model for Method 4 is <sim_att_dist_obs_tie_2025.slx>

clc;
clear all;
close all;

warning off

rng(12);

%% Leader reference trajectory parameters

P0 = [4; -6; 0];
R0 = 4;
R1 = 8;
PZ0 = P0(3);
PZ1 = 10;

Rx2 = 10;
Ry2 = 6;
Rz2 = 4;

tseq1 = 10;            %%%% Phase t = 0s - 10s: circular ascending motion
tseq12 = 10;           %%%% Phase t = 10s - 20s: first half of the figure-eight trajectory
tseq2 = 10;            %%%% Phase t = 20s - 30s: second half of the figure-eight trajectory

T0 = 0;
T1 = tseq1;
T12 = tseq1 + tseq12;
T2 = tseq1 + tseq12 + tseq2;
dt = 0.01;

acc0 = zeros(3, 1);
vel0 = zeros(3, 1);
pos0 = zeros(3, 1);

acc0(1, 1) = - ( 4 * pi * sin( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 - R1 ) ) / ( ( T0 - T1 ) ^ 2 ) ...
                        - ( 4 * ( pi ^ 2 ) * cos( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 + ( ( R0 - R1 ) * ( T0 - T0 ) ) / ( T0 - T1 ) ) ) / ( ( T0 - T1 ) ^ 2 );
acc0(2, 1) = ( 4 * pi * cos( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 - R1 ) ) / ( ( T0 - T1 ) ^ 2 ) ...
                        - ( 4 * ( pi ^ 2 ) * sin( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 + ( ( R0 - R1 ) * ( T0 - T0 ) ) / ( T0 - T1 ) ) ) / ( ( T0 - T1 ) ^ 2 );
acc0(3, 1) = 0;

vel0(1, 1) = ( cos( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 - R1 ) ) / ( T0 - T1 ) ...
                        - ( 2 * pi * sin( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 + ( ( R0 - R1 ) * ( T0 - T0 ) ) / ( T0 - T1 ) ) ) / ( T0 - T1 );
vel0(2, 1) = ( sin( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 - R1 ) ) / ( T0 - T1 ) ...
                        + ( 2 * pi * cos( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 + ( ( R0 - R1 ) * ( T0 - T0 ) ) / ( T0 - T1 ) ) ) / ( T0 - T1 );
vel0(3, 1) = ( P0(3) - PZ1 ) / ( T0 - T1 );

pos0(1, 1) = ( P0(1) - R0 ) + ( R0 + ( ( R1 - R0 ) / ( T1 - T0 ) ) * ( T0 - T0 ) ) * cos( ( 2 * pi / ( T1 - T0 ) ) * ( T0 - T0 ) );
pos0(2, 1) = P0(2) - ( R0 + ( ( R1 - R0 ) / ( T1 - T0 ) ) * ( T0 - T0 ) )  * sin( ( 2 * pi / ( T1 - T0 ) ) * ( T0 - T0 ) );
pos0(3, 1) = P0(3) + ( ( ( PZ1 - P0(3) ) / ( T1 - T0 ) ) * ( T0 - T0 ) );

%% Parameter settings for Nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC)
cS0 = diag( [4.5; 4.5; 4.5] );

pp0 = 8 / 11;
gamma0 = 20;
eps_s_bar = 0.03 * pi;
eps_s_bar_div_pi = eps_s_bar / pi;
k_J_0 = diag( [1; 1; 1] );

c_w_1 = diag( [20; 15; 15] );
c_w_2 = diag( [20; 15; 15] );
c_w_3 = diag( [2; 2; 2] );

beta_w_1 = 1.2;
beta_w_2 = 0.2;
mu_c_theta = 100;


[p_p0_0, q_p0_0] = rat( pp0 );
[p_2_p0, q_2_p0] = rat( 2 * pp0 );
[p_p0_plus_1, q_p0_plus_1] = rat( pp0 + 1 );
[p_p0_min_1, q_p0_min_1] = rat( pp0 - 1 );
[p_p0_min_2, q_p0_min_2] = rat( pp0 - 2 );
[p_1_min_p0, q_1_min_p0] = rat( 1 - pp0 );

[p_beta_w_1, q_beta_w_1] = rat( beta_w_1 );
[p_beta_w_2, q_beta_w_2] = rat( beta_w_2 );

thr_p_plus_1 = cal_frac_squ( 3, p_p0_plus_1, q_p0_plus_1 );

beta_Phi_1 = pp0 * cal_frac_squ( 3, p_p0_plus_1, q_p0_plus_1 ) ...
                        * cal_frac_squ( eps_s_bar / pi, p_p0_min_2, q_p0_min_2 ) ...
                        / ( 2 * ( 2 * pp0 - 1 ) * tanh( gamma0 * ( eps_s_bar / pi ) / 2 ) ) ...
                        - ( cal_frac_squ( 3, p_p0_plus_1, q_p0_plus_1 ) * gamma0 / ( 4 * ( 2 * pp0 -1 ) ) ) ...
                        * cal_frac_squ( eps_s_bar / pi, p_p0_min_1, q_p0_min_1 ) ...
                        * ( ( sech( gamma0 * ( eps_s_bar / pi ) / 2 ) / tanh( gamma0 * ( eps_s_bar / pi ) / 2 ) ) ^ 2 );

beta_Phi_2 = ( 1 - pp0 ) * cal_frac_squ( 3, p_p0_plus_1, q_p0_plus_1 ) ...
                        / ( 2 * ( 1 - 2 * pp0 ) * cal_frac_squ( eps_s_bar / pi, p_p0_0, q_p0_0 ) * tanh( gamma0 * ( eps_s_bar / pi ) / 2 ) ) ...
                        - ( cal_frac_squ( 3, p_p0_plus_1, q_p0_plus_1 ) * gamma0 / ( 4 * ( 1 - 2 * pp0 ) ) ) ...
                        * cal_frac_squ( eps_s_bar / pi, p_1_min_p0, q_1_min_p0 ) ...
                        * ( ( sech( gamma0 * ( eps_s_bar / pi ) / 2 ) / tanh( gamma0 * ( eps_s_bar / pi ) / 2 ) ) ^ 2 );

%% Parameter settings for the high-order nonlinear differentiator (HOND) 
%% for estimating the first-order derivative of the auxiliary attitude angular velocity tracking error

zeta_diff_sigma_bar_w = 0.3;
Lamd_diff_sigma_bar_w = 0.8;
c1_diff_sigma_bar_w = diag([12; 12; 12]);
c2_diff_sigma_bar_w = diag([12; 12; 12]);
c3_diff_sigma_bar_w = diag([15; 15; 15]);

%% Disturbance parameters for Rotational channel

dw_aa_t = 0.2 * ones(3, 5) + ( 1.0 - 0.2 ) * rand(3, 5);
dw_ff_t = [ 0.4 * ones(1, 5) + ( 1.0 - 0.4 ) * rand(1, 5); ...
                    0.4 * ones(1, 5) + ( 1.0 - 0.4 ) * rand(1, 5); ...
                    0.2 * ones(1, 5) + ( 0.6 - 0.2 ) * rand(1, 5) ];
dw_phi_t = ( - pi / 2 ) * ones(3, 5) + ( pi / 2 - ( - pi / 2 ) ) * rand(3, 5);

%% Parameter settings for the high-order nonlinear differentiator (HOND) 
%% for estimating the first-order and second-order derivatives of the control input

zeta_diff_ui = 0.05;
Lamd_diff_ui = 0.8;
c1_diff_ui = diag([25; 30; 20]);
c2_diff_ui = diag([50; 50; 10]);
c3_diff_ui = diag([15; 15; 10]);

%% State variable initialization

NF = 5;
g0 = 9.80663;
e3 = [0;0;1];

%%%% UAV mass initialization
mass_i = zeros(1,NF);
mass_i(1,1) = 0.35;
mass_i(1,2) = 0.35;
mass_i(1,3) = 0.35;
mass_i(1,4) = 0.35;
mass_i(1,5) = 0.35;

%%%% UAV Inertia matrix initialization
Ji = zeros(3,3,NF);

Ji(:,:,1) = [20, 2, 0.9; 2, 17, 0.5; 0.9, 0.5, 15] * 1e-00;
Ji(:,:,2) = [22, 1, 0.9; 1, 19, 0.5; 0.9, 0.5, 15] * 1e-00;
Ji(:,:,3) = [18, 1, 1.5; 1, 15, 0.5; 1.5, 0.5, 17] * 1e-00;
Ji(:,:,4) = [18, 1,    1; 1, 20, 0.5;    1, 0.5, 15] * 1e-00;
Ji(:,:,5) = [18, 1,    1; 1, 20, 0.5;    1, 0.5, 15] * 1e-00;

%%%% Initialize the UAV state variables:
%%%% quaternion and angular velocity
Qi_init(:,1) = [0.9110; 0.3; -0.2; 0.2];
Qi_init(:,2) = [0.9274; -0.1; 0.2; 0.3];
Qi_init(:,3) = [0.8185; 0.1; -0.4; 0.4];
Qi_init(:,4) = [0.8185; -0.4; -0.1; 0.4];
Qi_init(:,5) = [0.9274; 0.2; -0.1; 0.3];
wi_init(:,1) = [-0.5;0.5;-0.45];
wi_init(:,2) = [0.5;-0.3;0.1];
wi_init(:,3) = [0.1;0.6;-0.1];
wi_init(:,4) = [0.4;0.4;-0.5];
wi_init(:,5) = [0.4;-0.4;0.5];

dv0_init = [acc0(1); acc0(2); acc0(3)];
ui_init = zeros(3,NF);
for iiNF = 1:NF
    ui_init(:,iiNF) = -g0 * e3 + dv0_init(:);
end

%%%% Initialize the desired quaternion states
Qci_init = repmat([1;0;0;0],1,NF);

%%%% Initialize the desired attitude rotation command matrix
%%%% obtained from the desired quaternion
Ri_init = zeros(3,3,NF);
Rci_init = zeros(3,3,NF);
for iuav = 1:NF
    Ri_init(:, :, iuav) = cal_R_with_Q(Qi_init(:,iuav));
    Rci_init(:, :, iuav) = cal_R_with_Q(Qci_init(:,iuav));
end

%%%% Initialization of the commanded angular velocity
wci_init = zeros(3,5);
dwci_init = zeros(3,5);
for iuav = 1:NF
    [wci_init(:,iuav), dwci_init(:,iuav)] = cal_angrate_wc_wcdot(Rci_init(:,:,iuav), ui_init(:,iuav), zeros(3,1), zeros(3,1));
end

%%%% Initialize the quaternion attitude error and auxiliary attitude error
for iuav = 1:NF
    Qei_init(:,iuav) = cal_Q_mul_Q(cal_inv_Q(Qci_init(:,iuav)), Qi_init(:,iuav));
    Rei_init(:,:,iuav) = cal_R_with_Q(Qei_init(:,iuav));
    wei_init(:,iuav) = wi_init(:,iuav) - (Rei_init(:,:,iuav)') * wci_init(:,iuav);
    V_Phi_Rei_init(:,iuav) = cal_LOG_coord(Rei_init(:,:,iuav));
end

%% Comparison of disturbance observers for the rotational control systems

%% Method 1 : Proposed Method

clear h1w h2w h3w
clear mu_d_w alfa_w_1 alfa_w_2
clear p_alfaw1 q_alfaw1 p_alfaw2 q_alfaw2

h1w = diag([20.0; 20.0; 20.0]);
h2w = diag([30.0; 35.0; 35.0]);
h3w = diag([30.0; 25.0; 30.0]);

mu_d_w = 100;
alfa_w_1 = 1.2;
alfa_w_2 = 0.4;

[p_alfaw1, q_alfaw1] = rat(alfa_w_1);
[p_alfaw2, q_alfaw2] = rat(alfa_w_2);

%%%% Load and execute the simulation model
open_system('sim_dist_att_obs_paper.slx');
sim('sim_dist_att_obs_paper.slx',[0,30]);
save_system;
close_system;

NF = size( wei, 2 );
tLen = size( wei, 3 );

%%%% Custom disturbances defined in the simulation
dwi_x_paper = zeros(NF,tLen);
dwi_y_paper = zeros(NF,tLen);
dwi_z_paper = zeros(NF,tLen);

%%%% Estimated disturbance values
dwi_hat_x_paper = zeros(NF,tLen);
dwi_hat_y_paper = zeros(NF,tLen);
dwi_hat_z_paper = zeros(NF,tLen);

%%%% Disturbance estimation errors
dwi_tilt_x_paper = zeros(NF,tLen);
dwi_tilt_y_paper = zeros(NF,tLen);
dwi_tilt_z_paper = zeros(NF,tLen);

%%%% Collect simulation data
for tt = 1:tLen
    for iiFF = 1:NF
        dwi_x_paper(iiFF,tt) = dwi(1,iiFF,tt);
        dwi_y_paper(iiFF,tt) = dwi(2,iiFF,tt);
        dwi_z_paper(iiFF,tt) = dwi(3,iiFF,tt);
        dwi_hat_x_paper(iiFF,tt) = dwi_hat(1,iiFF,tt);
        dwi_hat_y_paper(iiFF,tt) = dwi_hat(2,iiFF,tt);
        dwi_hat_z_paper(iiFF,tt) = dwi_hat(3,iiFF,tt);
        dwi_tilt_x_paper(iiFF,tt) = dwi_tilt(1,iiFF,tt);
        dwi_tilt_y_paper(iiFF,tt) = dwi_tilt(2,iiFF,tt);
        dwi_tilt_z_paper(iiFF,tt) = dwi_tilt(3,iiFF,tt);
    end
end

clear tout dwi dwi_hat dwi_tilt
clear NF tLen

%% Method 2 from Paper : IEEE TCYB (2024)
%% < Disturbance Rejection Event-Triggered Robust Model Predictive Control
%% for Tracking of Constrained Uncertain Robotic Manipulators >

clear Krw

Krw = diag([50; 50; 50]);

%%%% Load and execute the simulation model
open_system('sim_att_dist_obs_tcyb.slx');
sim('sim_att_dist_obs_tcyb.slx',[0,30]);
save_system;
close_system;

NF = size( wei, 2 );
tLen = size( wei, 3 );

%%%% Custom disturbances defined in the simulation
dwi_x = zeros(NF,tLen);
dwi_y = zeros(NF,tLen);
dwi_z = zeros(NF,tLen);

%%%% Estimated disturbance values
dwi_hat_x_tcyb_2024 = zeros(NF,tLen);
dwi_hat_y_tcyb_2024 = zeros(NF,tLen);
dwi_hat_z_tcyb_2024 = zeros(NF,tLen);

%%%% Disturbance estimation errors
dwi_tilt_x_tcyb_2024 = zeros(NF,tLen);
dwi_tilt_y_tcyb_2024 = zeros(NF,tLen);
dwi_tilt_z_tcyb_2024 = zeros(NF,tLen);

%%%% Collect simulation data
for tt = 1:tLen
    for iiFF = 1:NF
        dwi_x(iiFF,tt) = dwi(1,iiFF,tt);
        dwi_y(iiFF,tt) = dwi(2,iiFF,tt);
        dwi_z(iiFF,tt) = dwi(3,iiFF,tt);
        dwi_hat_x_tcyb_2024(iiFF,tt) = dwi_hat(1,iiFF,tt);
        dwi_hat_y_tcyb_2024(iiFF,tt) = dwi_hat(2,iiFF,tt);
        dwi_hat_z_tcyb_2024(iiFF,tt) = dwi_hat(3,iiFF,tt);
        dwi_tilt_x_tcyb_2024(iiFF,tt) = dwi_tilt(1,iiFF,tt);
        dwi_tilt_y_tcyb_2024(iiFF,tt) = dwi_tilt(2,iiFF,tt);
        dwi_tilt_z_tcyb_2024(iiFF,tt) = dwi_tilt(3,iiFF,tt);
    end
end

clear tout dwi dwi_hat dwi_tilt
clear NF tLen

%% Method 3 from Paper : IEEE TIE (2020)
%% < Composite Disturbance Rejection Attitude Control 
%% for Quadrotor With Unknown Disturbance >

clear l1w l2w epw kaiw
clear Aw eig_Aw

l1w = [50; 50; 50];
l2w = [60; 60; 60];
epw = [1; 1; 1];
kaiw = [10; 10; 10];

Aw = [-l1w(1,1), 1; -l2w(1,1), 0];
eig_Aw = eig(Aw);

%%%% Load and execute the simulation model
open_system('sim_att_dist_obs_tie_2020.slx');
sim('sim_att_dist_obs_tie_2020.slx',[0,30]);
save_system;
close_system;

NF = size( wei, 2 );
tLen = size( wei, 3 );

%%%% Estimated disturbance values
dwi_hat_x_tie_2020 = zeros(NF,tLen);
dwi_hat_y_tie_2020 = zeros(NF,tLen);
dwi_hat_z_tie_2020 = zeros(NF,tLen);

%%%% Disturbance estimation errors
dwi_tilt_x_tie_2020 = zeros(NF,tLen);
dwi_tilt_y_tie_2020 = zeros(NF,tLen);
dwi_tilt_z_tie_2020 = zeros(NF,tLen);

%%%% Collect simulation data
for tt = 1:tLen
    for iiFF = 1:NF
        dwi_hat_x_tie_2020(iiFF,tt) = dwi_hat(1,iiFF,tt);
        dwi_hat_y_tie_2020(iiFF,tt) = dwi_hat(2,iiFF,tt);
        dwi_hat_z_tie_2020(iiFF,tt) = dwi_hat(3,iiFF,tt);
        dwi_tilt_x_tie_2020(iiFF,tt) = dwi_tilt(1,iiFF,tt);
        dwi_tilt_y_tie_2020(iiFF,tt) = dwi_tilt(2,iiFF,tt);
        dwi_tilt_z_tie_2020(iiFF,tt) = dwi_tilt(3,iiFF,tt);
    end
end

clear tout dwi dwi_hat dwi_tilt
clear NF tLen


%% Method 4 from Paper : IEEE TIE (2025)
%% < Fixed-Time Angle Tracking Control for Multi-DOF
%% Manipulator Driven by Pneumatic Artificial Muscles >

lam1w = [40; 40; 40];
lam2w = [50; 50; 50];
p1w = [1; 1; 1];
mu = 3/4;
[p_mu, q_mu] = rat(mu);
[p_2_mu, q_2_mu] = rat(2 - mu);
[p_2mu_1, q_2mu_1] = rat(2 * mu - 1);
[p_3_2mu, q_3_2mu] = rat(3 - 2 * mu);

%%%% Load and execute the simulation model
open_system('sim_att_dist_obs_tie_2025.slx');
sim('sim_att_dist_obs_tie_2025.slx',[0,30]);
save_system;
close_system;

NF = size( wei, 2 );
tLen = size( wei, 3 );

%%%% Estimated disturbance values
dwi_hat_x_tie_2025 = zeros(NF,tLen);
dwi_hat_y_tie_2025 = zeros(NF,tLen);
dwi_hat_z_tie_2025 = zeros(NF,tLen);

%%%% Disturbance estimation errors
dwi_tilt_x_tie_2025 = zeros(NF,tLen);
dwi_tilt_y_tie_2025 = zeros(NF,tLen);
dwi_tilt_z_tie_2025 = zeros(NF,tLen);

%%%% Collect simulation data
for tt = 1:tLen
    for iiFF = 1:NF
        dwi_hat_x_tie_2025(iiFF,tt) = dwi_hat(1,iiFF,tt);
        dwi_hat_y_tie_2025(iiFF,tt) = dwi_hat(2,iiFF,tt);
        dwi_hat_z_tie_2025(iiFF,tt) = dwi_hat(3,iiFF,tt);
        dwi_tilt_x_tie_2025(iiFF,tt) = dwi_tilt(1,iiFF,tt);
        dwi_tilt_y_tie_2025(iiFF,tt) = dwi_tilt(2,iiFF,tt);
        dwi_tilt_z_tie_2025(iiFF,tt) = dwi_tilt(3,iiFF,tt);
    end
end

clear dwi dwi_hat dwi_tilt
clear NF tLen


%% Plot results for comparison
clc;
close all;

if size(tout,2) == 1
    tout = tout';
end

kk_plot_disturb = 1;
kk_gain = 10;

t0 = 0;
t1 = 10;
t12 = 20;
t2 = 30;
dt = 0.01;
t0_t1_sample = round( ( t0:dt:t1 ) / dt + 1 );
t1_t12_sample = round( ( (t1+dt):dt:t12 ) / dt + 1 );
t12_t2_sample = round( ( (t12+dt):dt:(t2-dt) ) / dt + 1 );

plt_x_1_min = 14.0;
plt_x_1_max = 16.0;
plt_x_2_min = 24.0;
plt_x_2_max = 26.0;

plt_y_1_min = 12.0;
plt_y_1_max = 14.0;
plt_y_2_min = 22.0;
plt_y_2_max = 24.0;

plt_z_1_min = 14.0;
plt_z_1_max = 16.0;
plt_z_2_min = 24.0;
plt_z_2_max = 26.0;

figure(1)

subplot(311)
ppk(1) = plot(tout, dwi_x(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, dwi_hat_x_paper(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
ppk(3) = plot(tout, dwi_hat_x_tcyb_2024(kk_plot_disturb,:), ...
                        'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
ppk(4) = plot(tout, dwi_hat_x_tie_2020(kk_plot_disturb,:), ...
                        'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
ppk(5) = plot(tout, dwi_hat_x_tie_2025(kk_plot_disturb,:), ...
                        'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;

plot([plt_x_1_min, plt_x_1_min], [min( [ dwi_hat_x_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t1_t12_sample) ] ), ...
                                                            max( [ dwi_hat_x_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_x_1_min, plt_x_1_max], [min( [ dwi_hat_x_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t1_t12_sample) ] ), ...
                                                            min( [ dwi_hat_x_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_x_1_max, plt_x_1_max], [min( [ dwi_hat_x_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t1_t12_sample) ] ), ...
                                                            max( [ dwi_hat_x_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_x_1_min, plt_x_1_max], [max( [ dwi_hat_x_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t1_t12_sample) ] ), ...
                                                            max( [ dwi_hat_x_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;


plot([plt_x_2_min, plt_x_2_min], [min( [ dwi_hat_x_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t12_t2_sample) ] ), ...
                                                            max( [ dwi_hat_x_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_x_2_min, plt_x_2_max], [min( [ dwi_hat_x_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t12_t2_sample) ] ), ...
                                                            min( [ dwi_hat_x_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_x_2_max, plt_x_2_max], [min( [ dwi_hat_x_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t12_t2_sample) ] ), ...
                                                            max( [ dwi_hat_x_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_x_2_min, plt_x_2_max], [max( [ dwi_hat_x_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t12_t2_sample) ] ), ...
                                                            max( [ dwi_hat_x_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_x_tie_2025(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;


xlabel('time (s)');
ylabel('$$\it \hat {d}_{i,x}^{\varpi} \rm (rad/s)$$','interpreter','latex');
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$\it {d}_{i}^{\varpi}$$', '$$\rm FxTDO$$', ...
                        '$$\rm [49]$$', '$$\rm [50]$$', '$$\rm [51]$$'}, ...
                         'NumColumns',5,'interpreter','latex','location','best','box','off','color','none');
set(gca, 'ylim', [-1.0, 1.0] .* kk_gain);
 
%%%% 第一个子图上方显示图例
ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);

subplot(312)
plot(tout, dwi_y(kk_plot_disturb,:), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, dwi_hat_y_paper(kk_plot_disturb,:), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout, dwi_hat_y_tcyb_2024(kk_plot_disturb,:), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout, dwi_hat_y_tie_2020(kk_plot_disturb,:), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout, dwi_hat_y_tie_2025(kk_plot_disturb,:), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it \hat {d}_{i,y}^{\varpi} \rm (rad/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.8, 0.8] .* kk_gain);

plot([plt_y_1_min, plt_y_1_min], [min( [ dwi_hat_y_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t1_t12_sample) ] ), ...
                                                            max( [ dwi_hat_y_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_y_1_min, plt_y_1_max], [min( [ dwi_hat_y_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t1_t12_sample) ] ), ...
                                                            min( [ dwi_hat_y_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_y_1_max, plt_y_1_max], [min( [ dwi_hat_y_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t1_t12_sample) ] ), ...
                                                            max( [ dwi_hat_y_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_y_1_min, plt_y_1_max], [max( [ dwi_hat_y_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t1_t12_sample) ] ), ...
                                                            max( [ dwi_hat_y_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;


plot([plt_y_2_min, plt_y_2_min], [min( [ dwi_hat_y_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t12_t2_sample) ] ), ...
                                                            max( [ dwi_hat_y_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_y_2_min, plt_y_2_max], [min( [ dwi_hat_y_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t12_t2_sample) ] ), ...
                                                            min( [ dwi_hat_y_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_y_2_max, plt_y_2_max], [min( [ dwi_hat_y_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t12_t2_sample) ] ), ...
                                                            max( [ dwi_hat_y_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_y_2_min, plt_y_2_max], [max( [ dwi_hat_y_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t12_t2_sample) ] ), ...
                                                            max( [ dwi_hat_y_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_y_tie_2025(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;

subplot(313)
plot(tout, dwi_z(kk_plot_disturb,:), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, dwi_hat_z_paper(kk_plot_disturb,:), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout, dwi_hat_z_tcyb_2024(kk_plot_disturb,:), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout, dwi_hat_z_tie_2020(kk_plot_disturb,:), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout, dwi_hat_z_tie_2025(kk_plot_disturb,:), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it \hat {d}_{i,z}^{\varpi} \rm (rad/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.8, 1.0] .* kk_gain);


plot([plt_z_1_min, plt_z_1_min], [min( [ dwi_hat_z_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t1_t12_sample) ] ), ...
                                                            max( [ dwi_hat_z_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_z_1_min, plt_z_1_max], [min( [ dwi_hat_z_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t1_t12_sample) ] ), ...
                                                            min( [ dwi_hat_z_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_z_1_max, plt_z_1_max], [min( [ dwi_hat_z_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t1_t12_sample) ] ), ...
                                                            max( [ dwi_hat_z_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_z_1_min, plt_z_1_max], [max( [ dwi_hat_z_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t1_t12_sample) ] ), ...
                                                            max( [ dwi_hat_z_tcyb_2024(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t1_t12_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;


plot([plt_z_2_min, plt_z_2_min], [min( [ dwi_hat_z_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t12_t2_sample) ] ), ...
                                                            max( [ dwi_hat_z_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_z_2_min, plt_z_2_max], [min( [ dwi_hat_z_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t12_t2_sample) ] ), ...
                                                            min( [ dwi_hat_z_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_z_2_max, plt_z_2_max], [min( [ dwi_hat_z_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t12_t2_sample) ] ), ...
                                                            max( [ dwi_hat_z_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([plt_z_2_min, plt_z_2_max], [max( [ dwi_hat_z_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t12_t2_sample) ] ), ...
                                                            max( [ dwi_hat_z_tcyb_2024(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2020(1,t12_t2_sample), ...
                                                                        dwi_hat_z_tie_2025(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;



%%%% 缩放图
ax1_1 = axes('Position', [0.44, 0.75, 0.15, 0.08]);
[~, id_start] = min(abs(tout - plt_x_1_min));
[~, id_end] = min(abs(tout - plt_x_1_max));
axes(ax1_1)
plot(tout(1,id_start:id_end), dwi_x(kk_plot_disturb,id_start:id_end), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_x_paper(kk_plot_disturb,id_start:id_end), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_x_tcyb_2024(kk_plot_disturb,id_start:id_end), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_x_tie_2020(kk_plot_disturb,id_start:id_end), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_x_tie_2025(kk_plot_disturb,id_start:id_end), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
set(gca, 'xtick', [], 'ytick', []);


ax1_2 = axes('Position', [0.7, 0.75, 0.15, 0.08]);
[~, id_start] = min(abs(tout - plt_x_2_min));
[~, id_end] = min(abs(tout - plt_x_2_max));
axes(ax1_2)
plot(tout(1,id_start:id_end), dwi_x(kk_plot_disturb,id_start:id_end), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_x_paper(kk_plot_disturb,id_start:id_end), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_x_tcyb_2024(kk_plot_disturb,id_start:id_end), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_x_tie_2020(kk_plot_disturb,id_start:id_end), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_x_tie_2025(kk_plot_disturb,id_start:id_end), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
set(gca, 'xtick', [], 'ytick', []);


ax2_1 = axes('Position', [0.44, 0.45, 0.15, 0.08]);
[~, id_start] = min(abs(tout - plt_y_1_min));
[~, id_end] = min(abs(tout - plt_y_1_max));
axes(ax2_1)
plot(tout(1,id_start:id_end), dwi_y(kk_plot_disturb,id_start:id_end), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_y_paper(kk_plot_disturb,id_start:id_end), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_y_tcyb_2024(kk_plot_disturb,id_start:id_end), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_y_tie_2020(kk_plot_disturb,id_start:id_end), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_y_tie_2025(kk_plot_disturb,id_start:id_end), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
set(gca, 'xtick', [], 'ytick', []);


ax2_2 = axes('Position', [0.68, 0.55, 0.15, 0.08]);
[~, id_start] = min(abs(tout - plt_y_2_min));
[~, id_end] = min(abs(tout - plt_y_2_max));
axes(ax2_2)
plot(tout(1,id_start:id_end), dwi_y(kk_plot_disturb,id_start:id_end), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_y_paper(kk_plot_disturb,id_start:id_end), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_y_tcyb_2024(kk_plot_disturb,id_start:id_end), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_y_tie_2020(kk_plot_disturb,id_start:id_end), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_y_tie_2025(kk_plot_disturb,id_start:id_end), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
set(gca, 'xtick', [], 'ytick', []);


ax3_1 = axes('Position', [0.44, 0.14, 0.15, 0.08]);
[~, id_start] = min(abs(tout - plt_z_1_min));
[~, id_end] = min(abs(tout - plt_z_1_max));
axes(ax3_1)
plot(tout(1,id_start:id_end), dwi_z(kk_plot_disturb,id_start:id_end), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_z_paper(kk_plot_disturb,id_start:id_end), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_z_tcyb_2024(kk_plot_disturb,id_start:id_end), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_z_tie_2020(kk_plot_disturb,id_start:id_end), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_z_tie_2025(kk_plot_disturb,id_start:id_end), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
set(gca, 'xtick', [], 'ytick', []);


ax3_2 = axes('Position', [0.74, 0.14, 0.15, 0.08]);
[~, id_start] = min(abs(tout - plt_z_2_min));
[~, id_end] = min(abs(tout - plt_z_2_max));
axes(ax3_2)
plot(tout(1,id_start:id_end), dwi_z(kk_plot_disturb,id_start:id_end), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_z_paper(kk_plot_disturb,id_start:id_end), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_z_tcyb_2024(kk_plot_disturb,id_start:id_end), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_z_tie_2020(kk_plot_disturb,id_start:id_end), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dwi_hat_z_tie_2025(kk_plot_disturb,id_start:id_end), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
set(gca, 'xtick', [], 'ytick', []);


%% 
