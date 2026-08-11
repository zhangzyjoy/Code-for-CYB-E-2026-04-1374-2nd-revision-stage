%% Distributed observer for the followers' desired positions
%% Based on the IEEE TCYB (2023) Paper
%% < Fixed-Time Cooperative Tracking Control for Double-Integrator
%% Multiagent Systems: A Time-Based Generator Approach >

%% Run <init_tcyb_2023_obs.m> to initialize the parameters and system states
%% Run <sim_tcyb_2023_obs.slx> to execute the formation control simulation
%% Run <plot_tcyb_2023_obs.m> to plot the simulation results

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


%% Parameter settings according to topology

g0 = 9.80663;
e3 = [0;0;1];

Pij = diag( inv(Lbarij') * ones(NF,1) );
Qij = (1/2) * ( Pij * Lbarij + (Lbarij') * Pij );

lambda_max_P = max(eig(Pij));
lambda_min_Q = min(eig(Qij));

b1_low_bound = lambda_max_P / lambda_min_Q;


%% Parameter settings for distributed state observers

k0 = 2;
ts1 = 10;
ts2 = 20;
delta1 = 0.1;
delta2 = 0.1;
b1 = 6.0;
c1 = 6.0;

%%%% State Initialization
eta_est_err_init = zeros(3, NF) - repmat(vel0(:), 1, NF) - delta_v_i_0;
kei_est_err_init = zeros(3, NF) - repmat(pos0(:), 1, NF) - delta_p_i_0;

eta_est_err_tilt_init = zeros(3, NF);
ksi_est_err_tilt_init = zeros(3, NF);

for iiuav = 1:5
    eta_est_err_tilt_init(:,iiuav) = Mii(iiuav, iiuav) * ( zeros(3,1) - vel0(:) - delta_v_i_0(:,iiuav) );
    ksi_est_err_tilt_init(:,iiuav) = Mii(iiuav, iiuav) * ( zeros(3,1) - pos0(:) - delta_p_i_0(:,iiuav) );
    for jjuav = 1:5
        eta_est_err_tilt_init(:,iiuav) = eta_est_err_tilt_init(:,iiuav) + ...
                                                            Wij(iiuav, jjuav) * ( ( zeros(3,1) - delta_v_i_0(:,iiuav) ) - ( zeros(3,1) - delta_v_i_0(:,jjuav) ) );
        ksi_est_err_tilt_init(:,iiuav) = ksi_est_err_tilt_init(:,iiuav) + ...
                                                            Wij(iiuav, jjuav) * ( ( zeros(3,1) - delta_p_i_0(:,iiuav) ) - ( zeros(3,1) - delta_p_i_0(:,jjuav) ) );
    end
end

%% 
