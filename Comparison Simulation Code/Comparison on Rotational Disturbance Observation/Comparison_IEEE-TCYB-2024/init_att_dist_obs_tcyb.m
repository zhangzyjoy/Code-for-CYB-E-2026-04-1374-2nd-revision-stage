%% Comparative disturbance observer for the attitude dynamics
%% Based on the IEEE TCYB (2024) paper:
%% <Disturbance Rejection Event-Triggered Robust Model Predictive Control
%% for Tracking of Constrained Uncertain Robotic Manipulators>

%% Run <init_att_dist_obs_tcyb.m> to initialize the parameters and system states
%% Run <sim_att_dist_obs_tcyb.slx> to execute the attitude control simulation
%% Run <plot_att_dist_obs_tcyb.m> to plot the simulation results

clc;
clear all;
close all;

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

%% Parameter settings for nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC)
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

%% Parameter settings for rotational disturbance observer

Krw = diag([50; 50; 50]);

%% 用于从控制输入求解控制输入一阶/二阶导数的高阶滑模微分器参数

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

%% 
