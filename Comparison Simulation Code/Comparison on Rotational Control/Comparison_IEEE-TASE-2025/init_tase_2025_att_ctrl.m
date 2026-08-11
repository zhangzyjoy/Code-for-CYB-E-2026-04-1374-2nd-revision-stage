%% Attitude controller based on the IEEE TASE (2025) paper:
%% <Singularity-Free Low-Complexity Fault-Tolerant Prescribed Performance Control for Spacecraft Attitude Stabilization>

%% A fixed-time convergent disturbance observer is designed to compensate
%% for external disturbances
%% Corresponding Simulink model: <sim_tase_2025_att_ctrl_adjust_2.slx>
%% The differential equations are used to update wi, Qi, Qei, and V_Phi_Rei

%% Run <init_tase_2025_att_ctrl.m> to initialize parameters and system states
%% Run <sim_tase_2025_att_ctrl.slx> to perform the attitude control simulation
%% Run <plot_tase_2025_att_ctrl.m> to plot the simulation results

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

%% Parameter settings for fixed-time disturbance observer (FxTDO) in Rotational channel

h1w = diag([20.0; 20.0; 20.0]);
h2w = diag([30.0; 35.0; 35.0]);
h3w = diag([30.0; 25.0; 30.0]);

mu_d_w = 100;
alfa_w_1 = 1.2;
alfa_w_2 = 0.4;

[p_alfaw1, q_alfaw1] = rat(alfa_w_1);
[p_alfaw2, q_alfaw2] = rat(alfa_w_2);

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
Qi_init(:,1) = [0.9110; -0.3; 0.2; -0.2];
Qi_init(:,2) = [0.9274; -0.1; 0.2; -0.3];
Qi_init(:,3) = [0.9387; -0.15; 0.3; -0.08];
Qi_init(:,4) = [0.9608; -0.2; 0.15; -0.12];
Qi_init(:,5) = [0.9274; -0.2; 0.1; -0.3];
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

%%%% Initialize the commanded angular velocity of attitude
wci_init = zeros(3,5);
dwci_init = zeros(3,5);
for iuav = 1:NF
    [wci_init(:,iuav), dwci_init(:,iuav)] = cal_angrate_wc_wcdot(Rci_init(:,:,iuav), ui_init(:,iuav), zeros(3,1), zeros(3,1));
end

%%%% Initialize the quaternion attitude error and auxiliary quaternion error
for iuav = 1:NF
    Qei_init(:,iuav) = cal_Q_mul_Q(cal_inv_Q(Qci_init(:,iuav)), Qi_init(:,iuav));
    Rei_init(:,:,iuav) = cal_R_with_Q(Qei_init(:,iuav));
    wei_init(:,iuav) = wi_init(:,iuav) - (Rei_init(:,:,iuav)') * wci_init(:,iuav);
    V_Phi_Rei_init(:,iuav) = cal_LOG_coord(Rei_init(:,:,iuav));
end

%% Singularity-free low-complexity fault-tolerant prescribed performance control (PPC)

c1 = 2.5;
c2 = 2.0;
mu1 = 0.2;
mu2 = 0.2;
T_ij = repmat([60.0; 60.0; 60.0], 1, NF);
phi_ij_init = ones(3, NF);

m_p_ij = zeros(1, NF);
p_ij_upper = zeros(1, NF);
p_ij = zeros(1, NF);
k_1_ij_upper = zeros(3, NF);
k_1_ij_lower = zeros(3, NF);
k_1_ij_init = zeros(3, NF);
k_1_ij_hat_init = zeros(3, NF);
q_ij_hat_init = zeros(3, NF);
eta_1_ij_init = zeros(3, NF);
alfa_i_init = zeros(3, NF);
err_wi_init = zeros(3, NF);

k_2_ij_upper = zeros(3, NF);
k_2_ij_lower = zeros(3, NF);

k_2_ij_init = zeros(3, NF);
eta_2_ij_init = zeros(3, NF);
beta_ij_init = zeros(3, NF);

for iuav = 1:NF
    m_p_ij(iuav) = abs( Qi_init(2, iuav) ) + abs( Qi_init(3, iuav) ) + abs( Qi_init(4, iuav) );
    p_ij_upper(iuav) = ( -m_p_ij(iuav) + sqrt( ( m_p_ij(iuav) ^ 2 ) + 3 * ( Qi_init(1, iuav) ^ 2 ) ) ) / 3;
    
    p_ij(iuav) = p_ij_upper(iuav) * ( 10/5 );
    
    for jj = 1:3
        k_1_ij_upper(jj, iuav) = p_ij(iuav) + abs( Qi_init(jj + 1, iuav) );
        
        k_1_ij_lower(jj, iuav) = p_ij(iuav) * ( 1/5 );
        
        k_1_ij_init(jj, iuav) = k_1_ij_lower(jj, iuav) + ( k_1_ij_upper(jj, iuav) - k_1_ij_lower(jj, iuav) ) * phi_ij_init(jj);
        k_1_ij_hat_init(jj, iuav) = (1/2) * ( k_1_ij_init(jj, iuav) + k_1_ij_lower(jj, iuav) );
        if Qi_init(jj + 1, iuav) >= 0
            q_ij_hat_init(jj, iuav) = Qi_init(jj + 1, iuav) - (1/2) * ( k_1_ij_init(jj, iuav) - k_1_ij_lower(jj, iuav) );
        else
            q_ij_hat_init(jj, iuav) = Qi_init(jj + 1, iuav) - (1/2) * ( k_1_ij_lower(jj, iuav) - k_1_ij_init(jj, iuav) );
        end
        eta_1_ij_init(jj, iuav) = tan( (pi/2) * ( Qi_init(jj + 1, iuav) / k_1_ij_init(jj, iuav) ) );
    end
    alfa_i_init(:, iuav) = -c1 * inv( (1/2) * ( Qi_init(1, iuav) * eye(3) + cal_askew_mat( Qi_init(2:4, iuav) ) ) ) * eta_1_ij_init(:, iuav);
    err_wi_init(:, iuav) = wi_init(:, iuav) - alfa_i_init(:, iuav);
    for jj = 1:3
        k_2_ij_upper(jj, iuav) = abs( err_wi_init(jj, iuav) ) * ( 8/5 );
        k_2_ij_lower(jj, iuav) = abs( err_wi_init(jj, iuav) ) * ( 1/5 );
        
        k_2_ij_init(jj, iuav) = k_2_ij_lower(jj, iuav) + ( k_2_ij_upper(jj, iuav) - k_2_ij_lower(jj, iuav) ) * exp( -mu2 * 0 );
        eta_2_ij_init(jj, iuav) = tan( (pi/2) * ( err_wi_init(jj, iuav) / k_2_ij_init(jj, iuav) ) );
        beta_ij_init(jj, iuav) = ( 1 / k_2_ij_init(jj, iuav) ) * ( 1 / ( cos( (pi/2) * ( eta_2_ij_init(jj, iuav) / k_2_ij_init(jj, iuav) ) ) ^ 2 ) );
    end
end

%% 
