%% Translational control : comparison of external disturbance observers
%% The compared methods are listed as follows:

%% Method 1 : Proposed Method
%% The simulation model for Method 1 is <sim_dist_pos_obs_paper.slx>

%% Method 2 : IEEE TCAS-I (2023) < Composite Nonlinear Extended State Observer-Based Trajectory Tracking Control for Quadrotor Under Input Constraints >
%% The simulation model for Method 2 is < sim_pos_dist_obs_tcas.slx >

%% Method 3 : IEEE TCYB (2023) < Practical Predefined-Time Output-Feedback Consensus Tracking Control for Multiagent Systems >
%% The simulation model for Method 3 is < sim_pos_dist_obs_tcyb.slx >

%% Method 4 : IEEE TIE (2025) < Fixed-Time Formation-Containment of Nonlinear Systems Using Intermittent Output and Connectivity >
%% The simulation model for Method 4 is < sim_pos_dist_obs_tie.slx >

clc;
clear all;
close all;

warning off

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

tseq1 = 10;            %%%% Phase t = 0s - 10s : circular ascending motion
tseq12 = 10;           %%%% Phase t = 10s - 20s : first half of the figure-eight trajectory
tseq2 = 10;            %%%% Phase t = 20s - 30s : second half of the figure-eight trajectory

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

%% Distributed practical fixed-time distributed formation controller (PFxTDFC)

alfa_theta = 20;

beta1 = 1.2;
beta2 = 0.8;
[p_beta1, q_beta1] = rat(beta1);
[p_beta2, q_beta2] = rat(beta2);

dbeta_1 = beta1 - 1;
dbeta_2 = beta2 - 1;
[p_dbeta1, q_dbeta1] = rat(dbeta_1);
[p_dbeta2, q_dbeta2] = rat(dbeta_2);

kai_chi_1 = diag([6; 6; 6]);
kai_chi_2 = diag([6; 6; 6]);
kai_u_1 = diag([8; 8; 8]);
kai_u_2 = diag([8; 8; 8]);

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

%% Comparison of translational dynamics disturbance observer

%% Method 1 : proposed method

clear c1v c2v c3v alfa_v_1 alfa_v_2
clear p_alfav1 q_alfav1 p_alfav2 q_alfav2

%%%% Parameter settings for translational disturbance observer
c3v = diag([16.0; 16.0; 16.0]);
c1v = diag([16.0; 16.0; 16.0]);
c2v = diag([16.0; 16.0; 16.0]);

alfa_v_1 = 1.1;
alfa_v_2 = 0.2;
[p_alfav1, q_alfav1] = rat(alfa_v_1);
[p_alfav2, q_alfav2] = rat(alfa_v_2);

%%%% Load and execute the simulation model
open_system('sim_dist_pos_obs_paper.slx');
sim('sim_dist_pos_obs_paper.slx',[0,30]);
save_system;
close_system;

NF = size( ksi_track_err_vec, 2 );
tLen = size( ksi_track_err_vec, 3 );

%%%% Custom disturbances defined in the simulation
dp_x = zeros(NF,tLen);
dp_y = zeros(NF,tLen);
dp_z = zeros(NF,tLen);

%%%% Estimated disturbance values
dp_hat_x_paper = zeros(NF,tLen);
dp_hat_y_paper = zeros(NF,tLen);
dp_hat_z_paper = zeros(NF,tLen);

%%%% Disturbance estimation errors
dp_tilt_x_paper = zeros(NF,tLen);
dp_tilt_y_paper = zeros(NF,tLen);
dp_tilt_z_paper = zeros(NF,tLen);

%%%% Collect simulation data
for tt = 1:tLen
    for iiFF = 1:NF
        dp_x(iiFF,tt) = dp_vec(1,iiFF,tt);
        dp_y(iiFF,tt) = dp_vec(2,iiFF,tt);
        dp_z(iiFF,tt) = dp_vec(3,iiFF,tt);
        dp_hat_x_paper(iiFF,tt) = dp_hat_vec(1,iiFF,tt);
        dp_hat_y_paper(iiFF,tt) = dp_hat_vec(2,iiFF,tt);
        dp_hat_z_paper(iiFF,tt) = dp_hat_vec(3,iiFF,tt);
        dp_tilt_x_paper(iiFF,tt) = dp_tilt_vec(1,iiFF,tt);
        dp_tilt_y_paper(iiFF,tt) = dp_tilt_vec(2,iiFF,tt);
        dp_tilt_z_paper(iiFF,tt) = dp_tilt_vec(3,iiFF,tt);
    end
end

clear tout dp_vec dp_hat_vec dp_tilt_vec
clear NF tLen

%% Method 2 from Paper : IEEE TCAS-I (2023) 
%% < Composite Nonlinear Extended State Observer-Based Trajectory Tracking Control for Quadrotor Under Input Constraints >

clear k1v k2v k3v k0
clear ell1 ell2 ell3 epv alfa
clear p_alfa_1 q_alfa_1 p_alfa_2 q_alfa_2

k1v = [0.1; 0.1; 0.1];
k2v = [0.1; 0.1; 0.1];
k3v = [0.01; 0.01; 0.01];
k0 = [10; 10; 10];
ell1 = [100; 100; 100];
ell2 = [50; 50; 50];
ell3 = [5; 5; 5];
epv = [0.1; 0.1; 0.1];
alfa = 0.2;

[p_alfa_1, q_alfa_1] = rat(alfa);
[p_alfa_2, q_alfa_2] = rat(1 - alfa);

%%%% Load and execute the simulation model
open_system('sim_pos_dist_obs_tcas.slx');
sim('sim_pos_dist_obs_tcas.slx',[0,30]);
save_system;
close_system;

NF = size( ksi_track_err_vec, 2 );
tLen = size( ksi_track_err_vec, 3 );

%%%% Estimated disturbance values
dp_hat_x_tcas = zeros(NF,tLen);
dp_hat_y_tcas = zeros(NF,tLen);
dp_hat_z_tcas = zeros(NF,tLen);

%%%% Disturbance estimation errors
dp_tilt_x_tcas = zeros(NF,tLen);
dp_tilt_y_tcas = zeros(NF,tLen);
dp_tilt_z_tcas = zeros(NF,tLen);

%%%% Collect simulation data
for tt = 1:tLen
    for iiFF = 1:NF
        dp_hat_x_tcas(iiFF,tt) = dp_hat_vec(1,iiFF,tt);
        dp_hat_y_tcas(iiFF,tt) = dp_hat_vec(2,iiFF,tt);
        dp_hat_z_tcas(iiFF,tt) = dp_hat_vec(3,iiFF,tt);
        dp_tilt_x_tcas(iiFF,tt) = dp_tilt_vec(1,iiFF,tt);
        dp_tilt_y_tcas(iiFF,tt) = dp_tilt_vec(2,iiFF,tt);
        dp_tilt_z_tcas(iiFF,tt) = dp_tilt_vec(3,iiFF,tt);
    end
end

clear tout dp_vec dp_hat_vec dp_tilt_vec

%% Method 3 from Paper : IEEE TCYB (2023)
%% < Practical Predefined-Time Output-Feedback Consensus Tracking Control for Multiagent Systems >

clear gamma_0 gamma_obs_1 gamma_obs_2 gamma_obs_3 gamma_obs_4
clear tf2_zeta eps_zeta

gamma_0 = 20;
gamma_obs_1 = 4;
gamma_obs_2 = 4;
gamma_obs_3 = 4;
gamma_obs_4 = 4;

tf2_zeta = 2;
eps_zeta = 0.2;

%%%% Load and execute the simulation model
open_system('sim_pos_dist_obs_tcyb.slx');
sim('sim_pos_dist_obs_tcyb.slx',[0,30]);
save_system;
close_system;

NF = size( ksi_track_err_vec, 2 );
tLen = size( ksi_track_err_vec, 3 );

%%%% Estimated disturbance values
dp_hat_x_tcyb = zeros(NF,tLen);
dp_hat_y_tcyb = zeros(NF,tLen);
dp_hat_z_tcyb = zeros(NF,tLen);

%%%% Disturbance estimation errors
dp_tilt_x_tcyb = zeros(NF,tLen);
dp_tilt_y_tcyb = zeros(NF,tLen);
dp_tilt_z_tcyb = zeros(NF,tLen);

%%%% Collect simulation data
for tt = 1:tLen
    for iiFF = 1:NF
        dp_hat_x_tcyb(iiFF,tt) = dp_hat_vec(1,iiFF,tt);
        dp_hat_y_tcyb(iiFF,tt) = dp_hat_vec(2,iiFF,tt);
        dp_hat_z_tcyb(iiFF,tt) = dp_hat_vec(3,iiFF,tt);
        dp_tilt_x_tcyb(iiFF,tt) = dp_tilt_vec(1,iiFF,tt);
        dp_tilt_y_tcyb(iiFF,tt) = dp_tilt_vec(2,iiFF,tt);
        dp_tilt_z_tcyb(iiFF,tt) = dp_tilt_vec(3,iiFF,tt);
    end
end

clear tout dp_vec dp_hat_vec dp_tilt_vec


%% Method 4 : IEEE TIE (2025)
%% < Fixed-Time Formation-Containment of Nonlinear Systems
%% Using Intermittent Output and Connectivity >

clear kai1v kai2v kai3v Tu mv Lv
clear ell1v ell2v ell3v
clear p_m_1 q_m_1 p_m_2 q_m_2 p_m_3 q_m_3

kai1v = [20; 20; 20];
kai2v = [20; 20; 20];
kai3v = [20; 20; 20];
Tu = 0.4;
mv = 0.4;
Lv = 40;

ell1v = [2 * (Lv ^ (1/3)); 2 * (Lv ^ (1/3)); 2 * (Lv ^ (1/3))];
ell2v = [1.5 * (Lv ^ (2/3)); 1.5 * (Lv ^ (2/3)); 1.5 * (Lv ^ (2/3))];
ell3v = [1.1 * Lv; 1.1 * Lv; 1.1 * Lv];

[p_m_1, q_m_1] = rat((3 + mv) / 3);
[p_m_2, q_m_2] = rat((3 + 2 * mv) / 3);
[p_m_3, q_m_3] = rat(1 + mv);

%%%% Load and execute the simulation model
open_system('sim_pos_dist_obs_tie.slx');
sim('sim_pos_dist_obs_tie.slx',[0,30]);
save_system;
close_system;

NF = size( ksi_track_err_vec, 2 );
tLen = size( ksi_track_err_vec, 3 );

%%%% Estimated disturbance values
dp_hat_x_tie = zeros(NF,tLen);
dp_hat_y_tie = zeros(NF,tLen);
dp_hat_z_tie = zeros(NF,tLen);

%%%% Disturbance estimation errors
dp_tilt_x_tie = zeros(NF,tLen);
dp_tilt_y_tie = zeros(NF,tLen);
dp_tilt_z_tie = zeros(NF,tLen);

%%%% Collect simulation data
for tt = 1:tLen
    for iiFF = 1:NF
        dp_hat_x_tie(iiFF,tt) = dp_hat_vec(1,iiFF,tt);
        dp_hat_y_tie(iiFF,tt) = dp_hat_vec(2,iiFF,tt);
        dp_hat_z_tie(iiFF,tt) = dp_hat_vec(3,iiFF,tt);
        dp_tilt_x_tie(iiFF,tt) = dp_tilt_vec(1,iiFF,tt);
        dp_tilt_y_tie(iiFF,tt) = dp_tilt_vec(2,iiFF,tt);
        dp_tilt_z_tie(iiFF,tt) = dp_tilt_vec(3,iiFF,tt);
    end
end

clear dp_vec dp_hat_vec dp_tilt_vec

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

figure(1)

subplot(311)
ppk(1) = plot(tout, dp_x(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, dp_hat_x_paper(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
ppk(3) = plot(tout, dp_hat_x_tcas(kk_plot_disturb,:), ...
                        'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
ppk(4) = plot(tout, dp_hat_x_tcyb(kk_plot_disturb,:), ...
                        'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
ppk(5) = plot(tout, dp_hat_x_tie(kk_plot_disturb,:), ...
                        'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;

plot([14.0, 14.0], [min( [ dp_hat_x_tcas(1,t1_t12_sample), ...
                                            dp_hat_x_tcyb(1,t1_t12_sample), ...
                                            dp_hat_x_tie(1,t1_t12_sample) ] ), ...
                                max( [ dp_hat_x_tcas(1,t1_t12_sample), ...
                                            dp_hat_x_tcyb(1,t1_t12_sample), ...
                                            dp_hat_x_tie(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([14.0, 16.0], [min( [ dp_hat_x_tcas(1,t1_t12_sample), ...
                                            dp_hat_x_tcyb(1,t1_t12_sample), ...
                                            dp_hat_x_tie(1,t1_t12_sample) ] ), ...
                                min( [ dp_hat_x_tcas(1,t1_t12_sample), ...
                                            dp_hat_x_tcyb(1,t1_t12_sample), ...
                                            dp_hat_x_tie(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([16.0, 16.0], [min( [ dp_hat_x_tcas(1,t1_t12_sample), ...
                                            dp_hat_x_tcyb(1,t1_t12_sample), ...
                                            dp_hat_x_tie(1,t1_t12_sample) ] ), ...
                                max( [ dp_hat_x_tcas(1,t1_t12_sample), ...
                                            dp_hat_x_tcyb(1,t1_t12_sample), ...
                                            dp_hat_x_tie(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([14.0, 16.0], [max( [ dp_hat_x_tcas(1,t1_t12_sample), ...
                                            dp_hat_x_tcyb(1,t1_t12_sample), ...
                                            dp_hat_x_tie(1,t1_t12_sample) ] ), ...
                                max( [ dp_hat_x_tcas(1,t1_t12_sample), ...
                                            dp_hat_x_tcyb(1,t1_t12_sample), ...
                                            dp_hat_x_tie(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;


plot([24.0, 24.0], [min( [ dp_hat_x_tcas(1,t12_t2_sample), ...
                                            dp_hat_x_tcyb(1,t12_t2_sample), ...
                                            dp_hat_x_tie(1,t12_t2_sample) ] ), ...
                                max( [ dp_hat_x_tcas(1,t12_t2_sample), ...
                                            dp_hat_x_tcyb(1,t12_t2_sample), ...
                                            dp_hat_x_tie(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([24.0, 26.0], [min( [ dp_hat_x_tcas(1,t12_t2_sample), ...
                                            dp_hat_x_tcyb(1,t12_t2_sample), ...
                                            dp_hat_x_tie(1,t12_t2_sample) ] ), ...
                                min( [ dp_hat_x_tcas(1,t12_t2_sample), ...
                                            dp_hat_x_tcyb(1,t12_t2_sample), ...
                                            dp_hat_x_tie(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([26.0, 26.0], [min( [ dp_hat_x_tcas(1,t12_t2_sample), ...
                                            dp_hat_x_tcyb(1,t12_t2_sample), ...
                                            dp_hat_x_tie(1,t12_t2_sample) ] ), ...
                                max( [ dp_hat_x_tcas(1,t12_t2_sample), ...
                                            dp_hat_x_tcyb(1,t12_t2_sample), ...
                                            dp_hat_x_tie(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([24.0, 26.0], [max( [ dp_hat_x_tcas(1,t12_t2_sample), ...
                                            dp_hat_x_tcyb(1,t12_t2_sample), ...
                                            dp_hat_x_tie(1,t12_t2_sample) ] ), ...
                                max( [ dp_hat_x_tcas(1,t12_t2_sample), ...
                                            dp_hat_x_tcyb(1,t12_t2_sample), ...
                                            dp_hat_x_tie(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;


xlabel('time (s)');
ylabel('$$\it \hat {d}_{i,x}^{v} \rm (m/s)$$','interpreter','latex');
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$\it {d}_{i}^{v}$$', '$$\rm FxTDO$$', ...
                        '$$\rm [46]$$', '$$\rm [47]$$', '$$\rm [48]$$'}, ...
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
plot(tout, dp_y(kk_plot_disturb,:), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, dp_hat_y_paper(kk_plot_disturb,:), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout, dp_hat_y_tcas(kk_plot_disturb,:), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout, dp_hat_y_tcyb(kk_plot_disturb,:), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout, dp_hat_y_tie(kk_plot_disturb,:), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it \hat {d}_{i,y}^{v} \rm (m/s)$$','interpreter','latex');
set(gca, 'ylim', [-1.0, 1.0] .* kk_gain);

plot([14.0, 14.0], [min( [ dp_hat_y_tcas(1,t1_t12_sample), ...
                                            dp_hat_y_tcyb(1,t1_t12_sample), ...
                                            dp_hat_y_tie(1,t1_t12_sample) ] ), ...
                                max( [ dp_hat_y_tcas(1,t1_t12_sample), ...
                                            dp_hat_y_tcyb(1,t1_t12_sample), ...
                                            dp_hat_y_tie(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([14.0, 16.0], [min( [ dp_hat_y_tcas(1,t1_t12_sample), ...
                                            dp_hat_y_tcyb(1,t1_t12_sample), ...
                                            dp_hat_y_tie(1,t1_t12_sample) ] ), ...
                                min( [ dp_hat_y_tcas(1,t1_t12_sample), ...
                                            dp_hat_y_tcyb(1,t1_t12_sample), ...
                                            dp_hat_y_tie(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([16.0, 16.0], [min( [ dp_hat_y_tcas(1,t1_t12_sample), ...
                                            dp_hat_y_tcyb(1,t1_t12_sample), ...
                                            dp_hat_y_tie(1,t1_t12_sample) ] ), ...
                                max( [ dp_hat_y_tcas(1,t1_t12_sample), ...
                                            dp_hat_y_tcyb(1,t1_t12_sample), ...
                                            dp_hat_y_tie(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([14.0, 16.0], [max( [ dp_hat_y_tcas(1,t1_t12_sample), ...
                                            dp_hat_y_tcyb(1,t1_t12_sample), ...
                                            dp_hat_y_tie(1,t1_t12_sample) ] ), ...
                                max( [ dp_hat_y_tcas(1,t1_t12_sample), ...
                                            dp_hat_y_tcyb(1,t1_t12_sample), ...
                                            dp_hat_y_tie(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;


plot([24.0, 24.0], [min( [ dp_hat_y_tcas(1,t12_t2_sample), ...
                                            dp_hat_y_tcyb(1,t12_t2_sample), ...
                                            dp_hat_y_tie(1,t12_t2_sample) ] ), ...
                                max( [ dp_hat_y_tcas(1,t12_t2_sample), ...
                                            dp_hat_y_tcyb(1,t12_t2_sample), ...
                                            dp_hat_y_tie(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([24.0, 26.0], [min( [ dp_hat_y_tcas(1,t12_t2_sample), ...
                                            dp_hat_y_tcyb(1,t12_t2_sample), ...
                                            dp_hat_y_tie(1,t12_t2_sample) ] ), ...
                                min( [ dp_hat_y_tcas(1,t12_t2_sample), ...
                                            dp_hat_y_tcyb(1,t12_t2_sample), ...
                                            dp_hat_y_tie(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([26.0, 26.0], [min( [ dp_hat_y_tcas(1,t12_t2_sample), ...
                                            dp_hat_y_tcyb(1,t12_t2_sample), ...
                                            dp_hat_y_tie(1,t12_t2_sample) ] ), ...
                                max( [ dp_hat_y_tcas(1,t12_t2_sample), ...
                                            dp_hat_y_tcyb(1,t12_t2_sample), ...
                                            dp_hat_y_tie(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([24.0, 26.0], [max( [ dp_hat_y_tcas(1,t12_t2_sample), ...
                                            dp_hat_y_tcyb(1,t12_t2_sample), ...
                                            dp_hat_y_tie(1,t12_t2_sample) ] ), ...
                                max( [ dp_hat_y_tcas(1,t12_t2_sample), ...
                                            dp_hat_y_tcyb(1,t12_t2_sample), ...
                                            dp_hat_y_tie(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;

subplot(313)
plot(tout, dp_z(kk_plot_disturb,:), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout, dp_hat_z_paper(kk_plot_disturb,:), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout, dp_hat_z_tcas(kk_plot_disturb,:), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout, dp_hat_z_tcyb(kk_plot_disturb,:), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout, dp_hat_z_tie(kk_plot_disturb,:), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it \hat {d}_{i,z}^{v} \rm (m/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.8, 1.0] .* kk_gain);


plot([14.0, 14.0], [min( [ dp_hat_z_tcas(1,t1_t12_sample), ...
                                            dp_hat_z_tcyb(1,t1_t12_sample), ...
                                            dp_hat_z_tie(1,t1_t12_sample) ] ), ...
                                max( [ dp_hat_z_tcas(1,t1_t12_sample), ...
                                            dp_hat_z_tcyb(1,t1_t12_sample), ...
                                            dp_hat_z_tie(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([14.0, 16.0], [min( [ dp_hat_z_tcas(1,t1_t12_sample), ...
                                            dp_hat_z_tcyb(1,t1_t12_sample), ...
                                            dp_hat_z_tie(1,t1_t12_sample) ] ), ...
                                min( [ dp_hat_z_tcas(1,t1_t12_sample), ...
                                            dp_hat_z_tcyb(1,t1_t12_sample), ...
                                            dp_hat_z_tie(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([16.0, 16.0], [min( [ dp_hat_z_tcas(1,t1_t12_sample), ...
                                            dp_hat_z_tcyb(1,t1_t12_sample), ...
                                            dp_hat_z_tie(1,t1_t12_sample) ] ), ...
                                max( [ dp_hat_z_tcas(1,t1_t12_sample), ...
                                            dp_hat_z_tcyb(1,t1_t12_sample), ...
                                            dp_hat_z_tie(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([14.0, 16.0], [max( [ dp_hat_z_tcas(1,t1_t12_sample), ...
                                            dp_hat_z_tcyb(1,t1_t12_sample), ...
                                            dp_hat_z_tie(1,t1_t12_sample) ] ), ...
                                max( [ dp_hat_z_tcas(1,t1_t12_sample), ...
                                            dp_hat_z_tcyb(1,t1_t12_sample), ...
                                            dp_hat_z_tie(1,t1_t12_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;


plot([24.0, 24.0], [min( [ dp_hat_z_tcas(1,t12_t2_sample), ...
                                            dp_hat_z_tcyb(1,t12_t2_sample), ...
                                            dp_hat_z_tie(1,t12_t2_sample) ] ), ...
                                max( [ dp_hat_z_tcas(1,t12_t2_sample), ...
                                            dp_hat_z_tcyb(1,t12_t2_sample), ...
                                            dp_hat_z_tie(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([24.0, 26.0], [min( [ dp_hat_z_tcas(1,t12_t2_sample), ...
                                            dp_hat_z_tcyb(1,t12_t2_sample), ...
                                            dp_hat_z_tie(1,t12_t2_sample) ] ), ...
                                min( [ dp_hat_z_tcas(1,t12_t2_sample), ...
                                            dp_hat_z_tcyb(1,t12_t2_sample), ...
                                            dp_hat_z_tie(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([26.0, 26.0], [min( [ dp_hat_z_tcas(1,t12_t2_sample), ...
                                            dp_hat_z_tcyb(1,t12_t2_sample), ...
                                            dp_hat_z_tie(1,t12_t2_sample) ] ), ...
                                max( [ dp_hat_z_tcas(1,t12_t2_sample), ...
                                            dp_hat_z_tcyb(1,t12_t2_sample), ...
                                            dp_hat_z_tie(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;
plot([24.0, 26.0], [max( [ dp_hat_z_tcas(1,t12_t2_sample), ...
                                            dp_hat_z_tcyb(1,t12_t2_sample), ...
                                            dp_hat_z_tie(1,t12_t2_sample) ] ), ...
                                max( [ dp_hat_z_tcas(1,t12_t2_sample), ...
                                            dp_hat_z_tcyb(1,t12_t2_sample), ...
                                            dp_hat_z_tie(1,t12_t2_sample) ] ) ], 'color', 'k', 'linewidth', 1.0);
hold on;



%%%% 缩放图
ax1_1 = axes('Position', [0.44, 0.75, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 14.0));
[~, id_end] = min(abs(tout - 16.0));
axes(ax1_1)
plot(tout(1,id_start:id_end), dp_x(kk_plot_disturb,id_start:id_end), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_x_paper(kk_plot_disturb,id_start:id_end), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_x_tcas(kk_plot_disturb,id_start:id_end), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_x_tcyb(kk_plot_disturb,id_start:id_end), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_x_tie(kk_plot_disturb,id_start:id_end), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
set(gca, 'xtick', [], 'ytick', []);


ax1_2 = axes('Position', [0.7, 0.75, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 24.0));
[~, id_end] = min(abs(tout - 26.0));
axes(ax1_2)
plot(tout(1,id_start:id_end), dp_x(kk_plot_disturb,id_start:id_end), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_x_paper(kk_plot_disturb,id_start:id_end), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_x_tcas(kk_plot_disturb,id_start:id_end), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_x_tcyb(kk_plot_disturb,id_start:id_end), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_x_tie(kk_plot_disturb,id_start:id_end), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
set(gca, 'xtick', [], 'ytick', []);


ax2_1 = axes('Position', [0.44, 0.45, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 14.0));
[~, id_end] = min(abs(tout - 16.0));
axes(ax2_1)
plot(tout(1,id_start:id_end), dp_y(kk_plot_disturb,id_start:id_end), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_y_paper(kk_plot_disturb,id_start:id_end), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_y_tcas(kk_plot_disturb,id_start:id_end), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_y_tcyb(kk_plot_disturb,id_start:id_end), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_y_tie(kk_plot_disturb,id_start:id_end), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
set(gca, 'xtick', [], 'ytick', []);


ax2_2 = axes('Position', [0.68, 0.55, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 24.0));
[~, id_end] = min(abs(tout - 26.0));
axes(ax2_2)
plot(tout(1,id_start:id_end), dp_y(kk_plot_disturb,id_start:id_end), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_y_paper(kk_plot_disturb,id_start:id_end), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_y_tcas(kk_plot_disturb,id_start:id_end), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_y_tcyb(kk_plot_disturb,id_start:id_end), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_y_tie(kk_plot_disturb,id_start:id_end), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
set(gca, 'xtick', [], 'ytick', []);


ax3_1 = axes('Position', [0.44, 0.14, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 14.0));
[~, id_end] = min(abs(tout - 16.0));
axes(ax3_1)
plot(tout(1,id_start:id_end), dp_z(kk_plot_disturb,id_start:id_end), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_z_paper(kk_plot_disturb,id_start:id_end), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_z_tcas(kk_plot_disturb,id_start:id_end), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_z_tcyb(kk_plot_disturb,id_start:id_end), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_z_tie(kk_plot_disturb,id_start:id_end), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
set(gca, 'xtick', [], 'ytick', []);


ax3_2 = axes('Position', [0.74, 0.14, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 24.0));
[~, id_end] = min(abs(tout - 26.0));
axes(ax3_2)
plot(tout(1,id_start:id_end), dp_z(kk_plot_disturb,id_start:id_end), ...
            'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_z_paper(kk_plot_disturb,id_start:id_end), ...
            'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_z_tcas(kk_plot_disturb,id_start:id_end), ...
            'color', 'g', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_z_tcyb(kk_plot_disturb,id_start:id_end), ...
            'color', 'm', 'linestyle', '--', 'linewidth', 1.2);
hold on;
plot(tout(1,id_start:id_end), dp_hat_z_tie(kk_plot_disturb,id_start:id_end), ...
            'color', 'c', 'linestyle', '-.', 'linewidth', 1.2);
hold on;
set(gca, 'xtick', [], 'ytick', []);


%% 
