%% Leader-follower formation control
%% Distributed observer for estimating the desired position of followers
%% Differentiator-based velocity estimation feedback + external disturbance observer

%% Independent tests:
%% 1) Evaluate the position/velocity state estimation performance of the differentiator
%% 2) Evaluate the formation control accuracy of the distributed controller
%% The controller is designed with reference to the IEEE TASE (2025) paper:
%% "L1 Adaptive Control-Based Formation Tracking of Multiple Quadrotors
%% Without Linear Velocity Feedback Under Unknown Disturbances"

%% Run <init_tase_2025_pos_ctrl.m> to initialize parameters and system states
%% Run <sim_tase_2025_pos_ctrl.slx> to execute the simulation model
%% Run <plot_tase_2025_pos_ctrl.m> to plot the simulation results

clc;
clear all;
close all;

rng(12);

%% <1 leader + 5 followers> directed communication topology

NF = 5;

Wij = [0,0,1,0,0; ...
            0,0,1,1,0; ...
            1,0,0,0,1; ...
            0,1,1,0,0; ...
            0,0,1,0,0];
Mii = diag([1;0;0;0;1]);

Lij = diag(sum(Wij,2));
Lij = Lij - Wij;
Lbarij = Lij + Mii;

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


%% Desired formation configuration parameters of followers

X1 = 8;
X2 = 2;
X3 = 3;
Y1 = 10;
Y2 = 2;
Y3 = 4;
Z1 = 4;
Z2 = -6;
Z3 = 2;
theta0 = ( 2 / 5 ) * pi;
kai_1_t = 0.02;
kai_3_t = 0.1;

delta_v_i_0 = zeros(3, NF);
delta_v_i_0(:,1) = kai_1_t * [ X2; Y2; Z2 ];
delta_v_i_0(:,2) = kai_1_t * [ -X2; Y2; -Z2 ];
delta_v_i_0(:,3) = kai_1_t * [ X2; -Y2; -Z2 ];
delta_v_i_0(:,4) = kai_1_t * [ -X2; -Y2; -Z2 ];
delta_v_i_0(:,5) = kai_1_t * [ X2; -Y2; Z2 ];

delta_p_i_0 = zeros(3, NF);
delta_p_i_0(:,1) = [ X1 * cos( theta0 ) - X2; Y1 * sin( theta0 ) - Y2; Z1 - Z2 ];
delta_p_i_0(:,2) = [ -X1 * cos( theta0 ) + X2; Y1 * sin( theta0 ) - Y2; Z1 + Z2 ];
delta_p_i_0(:,3) = [ X1 - X2; Y2; Z1 * ( 3 / 2 ) - Z2 ];
delta_p_i_0(:,4) = [ -X1 * cos( theta0 ) + X2; -Y1 * sin( theta0 ) + Y2; Z1 + Z2 ];
delta_p_i_0(:,5) = [ X1 * cos( theta0 ) - X2; -Y1 * sin( theta0 ) + Y2; Z1 - Z2 ];

%% Initial position settings of followers

ksii_0 = zeros(3,5);
etai_0 = zeros(3,5);

ksii_0(:,1) = [-15; -15; -1];
ksii_0(:,2) = [-10; -5; 1];
ksii_0(:,3) = [-5; -5; -1];
ksii_0(:,4) = [-15; -5; 1];
ksii_0(:,5) = [-10; -15; -1];

etai_0(:,1) = [2; 2; 1];
etai_0(:,2) = [2; 2; 1];
etai_0(:,3) = [2; 2; 1];
etai_0(:,4) = [2; 2; 1];
etai_0(:,5) = [2; 2; 1];


%% Parameter settings for practical fixed-time distributed state observers (PFxTDSO)

gamma1_pos = 1.2;
gamma2_pos = 0.8;
gamma1_vel = 1.2;
gamma2_vel = 0.8;

[p_gamma1_pos, q_gamma1_pos] = rat(gamma1_pos);
[p_gamma2_pos, q_gamma2_pos] = rat(gamma2_pos);
[p_gamma1_vel, q_gamma1_vel] = rat(gamma1_vel);
[p_gamma2_vel, q_gamma2_vel] = rat(gamma2_vel);


alfa_theta_obs = 100;
l1p = diag([20; 20; 20]);
l2p = diag([50; 50; 50]);
l1v = diag([20; 20; 20]);
l2v = diag([30; 30; 30]);

eta_est_err_init = zeros(3, NF) - repmat(vel0(:), 1, NF) - delta_v_i_0;
kei_est_err_init = zeros(3, NF) - repmat(pos0(:), 1, NF) - delta_p_i_0;

eta_est_err_tilt_init = zeros(3, NF);
ksi_est_err_tilt_init = zeros(3, NF);

for iiuav = 1:5
    eta_est_err_tilt_init(:,iiuav) = Mii(iiuav, iiuav) * (zeros(3,1) - vel0(:) - delta_v_i_0(:,iiuav));
    ksi_est_err_tilt_init(:,iiuav) = Mii(iiuav, iiuav) * (zeros(3,1) - pos0(:) - delta_p_i_0(:,iiuav));
    for jjuav = 1:5
        eta_est_err_tilt_init(:,iiuav) = eta_est_err_tilt_init(:,iiuav) + ...
                                                            Wij(iiuav, jjuav) * ( (zeros(3,1) - delta_v_i_0(:,iiuav)) - (zeros(3,1) - delta_v_i_0(:,jjuav)) );
        ksi_est_err_tilt_init(:,iiuav) = ksi_est_err_tilt_init(:,iiuav) + ...
                                                            Wij(iiuav, jjuav) * ( (zeros(3,1) - delta_p_i_0(:,iiuav)) - (zeros(3,1) - delta_p_i_0(:,jjuav)) );
    end
end

%% Disturbance parameters for Translational channel

dp_aa_t = 0.2 * ones(3, 5) + ( 1.0 - 0.2 ) * rand(3, 5);
dp_ff_t = [ 0.4 * ones(1, 5) + ( 1.0 - 0.4 ) * rand(1, 5); ...
                    0.4 * ones(1, 5) + ( 1.0 - 0.4 ) * rand(1, 5); ...
                    0.2 * ones(1, 5) + ( 0.6 - 0.2 ) * rand(1, 5) ];
dp_phi_t = ( - pi / 2 ) * ones(3, 5) + ( pi / 2 - ( - pi / 2 ) ) * rand(3, 5);

%% Parameter settings for high-order nonlinear differentiator (HOND)
%% for solving the translational-channel disturbance observer auxiliary system

zeta_diff_sigma_bar_v = 0.08;
Lamd_diff_sigma_bar_v = 0.5;
c1_diff_sigma_bar_v = diag([25; 30; 20]);
c2_diff_sigma_bar_v = diag([50; 50; 10]);
c3_diff_sigma_bar_v = diag([15; 15; 10]);

%% Parameter settings for fixed-time disturbance observer (FxTDO) on translational channel

alfa_theta = 20;

c3v = diag([4.8; 6.4; 6.0]);
c1v = diag([5.8; 6.5; 6.5]);
c2v = diag([5.8; 6.5; 6.5]);

alfa_v_1 = 1.1;
alfa_v_2 = 0.2;
[p_alfav1, q_alfav1] = rat(alfa_v_1);
[p_alfav2, q_alfav2] = rat(alfa_v_2);

%% L1 Adaptive Based Formation Tracking Controller
%% & Adaptive disturbance observer

kp = 20;
kv = 20;
wp = 10;
Aps = -diag([5, 5, 5]);
tTs = 0.1;
Phi_inv = inv( Aps ) * ( exp( Aps * tTs ) - eye(3) );

%% High-order sliding-mode differentiator systems for velocity estimation

k_rho_1 = 10;
k_rho_2 = 10;
TTb = 2;
theta_1 = 0;
theta_2 = 1;
gamma_diff = 0.05;
gamma_diff_1 = gamma_diff + 1;
gamma_diff_2_2 = ( 2 + gamma_diff ) / 2;
[p_gamma_diff_1, q_gamma_diff_1] = rat( gamma_diff_1 );
[p_gamma_diff_2_2, q_gamma_diff_2_2] = rat( gamma_diff_2_2 );


%% Parameter settings of the high-order nonlinear differentiator (HOND)
%% for estimating the first-order and second-order derivatives of the control input

zeta_diff_ui = 0.05;
Lamd_diff_ui = 0.8;
c1_diff_ui = diag([25; 30; 20]);
c2_diff_ui = diag([50; 50; 10]);
c3_diff_ui = diag([15; 15; 10]);

%% State variable initialization

g0 = 9.80663;
e3 = [0;0;1];

%%%% UAV mass initialization
mass_i = zeros(1,NF);
mass_i(1,1) = 0.35;
mass_i(1,2) = 0.35;
mass_i(1,3) = 0.35;
mass_i(1,4) = 0.35;
mass_i(1,5) = 0.35;


%%%% Initialize the UAV state variables:
%%%% quaternion + angular velocity
Qi_init(:,1) = [0.9110; 0.3; -0.2; 0.2];
Qi_init(:,2) = [0.9274; -0.1; 0.2; 0.3];
Qi_init(:,3) = [0.8185; 0.1; -0.4; 0.4];
Qi_init(:,4) = [0.8185; -0.4; -0.1; 0.4];
Qi_init(:,5) = [0.9274; 0.2; -0.1; 0.3];


dv0_init = [acc0(1); acc0(2); acc0(3)];
ui_init = zeros(3,NF);
for iiNF = 1:NF
    ui_init(:,iiNF) = -g0 * e3 + dv0_init(:);
end

%%%% Initialize the desired quaternion states
Qci_init = repmat([1;0;0;0],1,NF);

%%%% Initialize the desired attitude rotation command matrix
%%%% calculated from the desired quaternion
Ri_init = zeros(3,3,NF);
Rci_init = zeros(3,3,NF);

for iuav = 1:NF
    Ri_init(:, :, iuav) = cal_R_with_Q(Qi_init(:,iuav));
    Rci_init(:, :, iuav) = cal_R_with_Q(Qci_init(:,iuav));
end

%%%% Initialize the commanded angular velocity of attitude
wci_init = zeros(3,5);
dwci_init = zeros(3,5);
for iuav = 1:NF
    [wci_init(:,iuav), dwci_init(:,iuav)] = cal_angrate_wc_wcdot(Rci_init(:,:,iuav), ui_init(:,iuav), zeros(3,1), zeros(3,1));
end

%% 
