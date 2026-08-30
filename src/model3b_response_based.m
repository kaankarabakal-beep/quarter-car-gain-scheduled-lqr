%% MODEL 3B - RESPONSE-BASED GAIN-SCHEDULED ACTIVE SUSPENSION
%
% Clean candidate version
% - Fast wheel RMS is used only to START a transient Protection event.
% - A transient receives 60 ms Protection.
% - If a shock is confirmed, Comfort follows.
% - Sustained high-frequency activity is handled by the slower normal RMS.
% - The same scheduler function is used for the nominal road and robustness tests.
% - Low Force is reserved for high-confidence calm/economy operation.
% - Comfort is used for body-dominated response with low wheel activity.
% - Balanced is the main mixed / ambiguous-dynamics controller.
% - Protection is used for sustained wheel-risk and fast transients.
% - Model 3B V1 scheduler logic is treated as LOCKED below.
% - Physical actuator force limit is now LOCKED at +/-750 N.
% - Physical actuator force-rate limit is now LOCKED at 50,000 N/s.
% - Sensor-noise robustness and shock confirmation are LOCKED.
% - Sensor timing is LOCKED at 500 Hz sampling and 4 ms transport delay.
% - Shock confirmation: 0.80 m/s for 3 consecutive sensor samples.
% - Final realism configuration is LOCKED.
% - Combined realism validation remains in this file for final verification.
% - Historical saturation sweep tables are removed; output stays compact.

clear;
clc;
close all;

show_figures = true;
show_tables = true;


%% 1. QUARTER-CAR PLANT

ms = 300;        % Sprung mass [kg]
mu = 40;         % Unsprung mass [kg]
ks = 22000;      % Suspension stiffness [N/m]
cs = 1500;       % Suspension damping [N*s/m]
kt = 200000;     % Tire stiffness [N/m]

A = [0,        1,             0,               0;
    -ks/ms,   -cs/ms,         ks/ms,           cs/ms;
     0,        0,             0,               1;
     ks/mu,    cs/mu,        -(ks+kt)/mu,     -cs/mu];

B = [0;
     1/ms;
     0;
    -1/mu];

E = [0;
     0;
     0;
     kt/mu];

x0 = [0; 0; 0; 0];


%% 2. LQR CONTROLLER BANK

Ca = [-ks/ms, -cs/ms, ks/ms, cs/ms];
Da = 1/ms;
Cs = [1, 0, -1, 0];

controller_names = [ ...
    "Balanced"
    "Comfort"
    "Low Force"
    "Protection"
    ];

a_ref_bank = [2.0; 1.2; 2.0; 3.0];
s_ref_bank = [0.020; 0.030; 0.020; 0.012];
F_ref_bank = [500; 800; 350; 300];

nK = length(controller_names);
K_bank = zeros(nK,4);

for j = 1:nK

    wa = 1/a_ref_bank(j)^2;
    ws = 1/s_ref_bank(j)^2;
    wF = 1/F_ref_bank(j)^2;

    Q = wa*(Ca'*Ca) + ws*(Cs'*Cs);
    N = wa*(Ca'*Da);
    R = wa*(Da^2) + wF;

    K_bank(j,:) = lqr(A,B,Q,R,N);

end


%% 3. RESPONSE-BASED SCHEDULER SETTINGS

Ts = 0.002;                  % 500 Hz sampling

window_time = 0.25;         % Normal RMS window [s]
window_samples = round(window_time/Ts);

fast_window_time = 0.05;    % Fast event-detection window [s]
fast_window_samples = round(fast_window_time/Ts);

protection_hold_time = 0.060;   % 60 ms transient Protection
protection_hold_samples = round(protection_hold_time/Ts);

shock_hold_time = 0.40;          % Comfort follow-up after shock [s]
shock_hold_samples = round(shock_hold_time/Ts);

% Normal classifier thresholds
%
% Low Force is now an economy mode:
% it is allowed only when body motion, suspension travel,
% AND wheel activity are all clearly non-critical.
Body_Low_Threshold = 0.30;           % [m/s^2]
Travel_Low_Threshold = 0.003;        % [m]
Wheel_LowForce_Max = 15.0;           % [m/s^2]

% Comfort is reserved for body-dominated response:
% body/travel is elevated while wheel activity remains low.
Wheel_Comfort_Max = 3.0;             % [m/s^2]
Travel_Comfort_Min = 0.004;          % [m]

% Sustained mechanical / wheel-risk region.
Wheel_High_Threshold = 20.0;         % [m/s^2]

% Fast transient detector.
Wheel_Fast_Protection_Threshold = 15.0; % [m/s^2]

% Main shock EVENT-entry threshold.
% This remains unchanged from Model 3B V1.
Shock_RelVel_Threshold = 0.88;       % [m/s]

% LOCKED: noise-robust shock confirmation.
%
% Data-derived from the threshold-characterization study:
% at 0.80 m/s, the nominal bump remains robustly confirmed under
% the tested Ideal / Low / Moderate / High-noise conditions,
% while nominal wheel-hop remains unconfirmed.
%
% Three consecutive samples gives one-sample margin:
%       3 samples x 2 ms = 6 ms.
Shock_Confirm_RelVel_Threshold = 0.80;   % [m/s]  FINAL LOCKED
Shock_Confirm_Samples = 3;               % consecutive samples


%% 3.1 LOCKED ACTUATOR LIMITS

% Force magnitude limit selected from the previous sensitivity study.
Fmax_locked = 750;                  % [N]

% Force-rate limit selected from the previous sensitivity study.
% At Ts = 0.002 s this allows at most 100 N force change per sample.
Fdotmax_locked = 50000;             % [N/s]


%% 3.2 SENSOR-MEASUREMENT BASELINE

% Main V1 results remain noise-free.
% Noise is added only in the dedicated sensitivity study.
%
% Noise acts only on the measurements used by the scheduler.
% True plant states/responses remain available for performance evaluation.

noise_free = struct( ...
    'bodyAccelStd',0, ...
    'wheelAccelStd',0, ...
    'travelStd',0, ...
    'relVelStd',0);


%% 3.3 LOCKED SENSOR TIMING

% Selected from the completed timing sensitivity studies.
%
% Sampling:
%       500 Hz -> 2 ms new measurement period
%
% Transport latency:
%       4 ms -> 2 base simulation samples
%
% 4 ms was the largest tested delay that preserved nominal transient
% peak behavior essentially unchanged. 500 Hz is retained because the
% locked 3-sample shock-confirmation logic degraded at lower tested rates.

SensorSamplingPeriod_locked = Ts;               % [s] = 0.002
SensorSamplingRate_locked = 1/Ts;              % [Hz] = 500

SensorDelay_locked_ms = 4;                     % [ms]
SensorDelaySamples_locked = ...
    round((SensorDelay_locked_ms/1000)/Ts);    % = 2 samples

SensorUpdateSamples_locked = 1;                % new measurement every 2 ms





%% 4. NOMINAL COMPOSITE ROAD

v = 10;                 % [m/s]
road_length = 80;       % [m]
lambda_wh = v/11.25;

zr_composite = @(x) compositeRoad(x, lambda_wh);


%% 5. MODEL 3B - RESPONSE-BASED SCHEDULER

sim3B = simulateResponseScheduler( ...
    v, ...
    road_length, ...
    zr_composite, ...
    Ts, ...
    x0, ...
    A, B, E, ...
    K_bank, ...
    ms, mu, ks, cs, kt, ...
    window_samples, ...
    fast_window_samples, ...
    protection_hold_samples, ...
    shock_hold_samples, ...
    Body_Low_Threshold, ...
    Travel_Low_Threshold, ...
    Wheel_LowForce_Max, ...
    Wheel_Comfort_Max, ...
    Wheel_High_Threshold, ...
    Travel_Comfort_Min, ...
    Wheel_Fast_Protection_Threshold, ...
    Shock_RelVel_Threshold, ...
    Shock_Confirm_RelVel_Threshold, ...
    Shock_Confirm_Samples, ...
    Fmax_locked, ...
    Fdotmax_locked, ...
    noise_free, ...
    SensorDelaySamples_locked, ...
    SensorUpdateSamples_locked);


%% 6. MODEL 3A - MANUAL SCHEDULED REFERENCE

sim3A = simulateManualReference( ...
    v, ...
    road_length, ...
    zr_composite, ...
    Ts, ...
    x0, ...
    A, B, E, ...
    K_bank, ...
    ms, mu, ks, cs, kt);


%% 7. GLOBAL PERFORMANCE SUMMARY

System = [ ...
    "Model 3A Manual (ideal)"
    "Model 3B V1 Locked Realism"
    ];

Accel_RMS = [ ...
    rms(sim3A.BodyAccel)
    rms(sim3B.BodyAccel)
    ];

Accel_Peak = [ ...
    max(abs(sim3A.BodyAccel))
    max(abs(sim3B.BodyAccel))
    ];

SuspTravel_Peak_mm = 1000*[ ...
    max(abs(sim3A.SuspTravel))
    max(abs(sim3B.SuspTravel))
    ];

TireDef_Peak_mm = 1000*[ ...
    max(abs(sim3A.TireDef))
    max(abs(sim3B.TireDef))
    ];

Force_RMS = [ ...
    rms(sim3A.Fa)
    rms(sim3B.Fa)
    ];

Force_Peak = [ ...
    max(abs(sim3A.Fa))
    max(abs(sim3B.Fa))
    ];

GlobalComparison = table( ...
    System, ...
    Accel_RMS, ...
    Accel_Peak, ...
    SuspTravel_Peak_mm, ...
    TireDef_Peak_mm, ...
    Force_RMS, ...
    Force_Peak);

if show_tables
    disp(' ')
    disp('--- GLOBAL PERFORMANCE ---')
    disp(GlobalComparison)
end


%% 8. KEY FIGURE 1 - ROAD AND CONTROLLER SELECTION

if show_figures

    figure('Name','Main Scheduling Comparison', ...
        'Position',[100 100 1100 650])

    % Small vertical offsets are used only for visualization
    % so that overlapping Model 3A and Model 3B selections remain visible.
    mode3A_plot = sim3A.mode - 0.06;
    mode3B_plot = sim3B.mode + 0.06;

    segment_edges = [16 32 44 54 64 72];


    % =====================================================
    % TOP: MODEL 3A vs MODEL 3B CONTROLLER SELECTION
    % =====================================================

    ax1 = subplot(2,1,1);

    stairs( ...
        sim3A.road_position, ...
        mode3A_plot, ...
        '--', ...
        'LineWidth',1.4, ...
        'DisplayName','Model 3A');

    hold on

    stairs( ...
        sim3B.road_position, ...
        mode3B_plot, ...
        'LineWidth',1.6, ...
        'DisplayName','Model 3B');

    for i = 1:length(segment_edges)

        xline( ...
            segment_edges(i), ...
            '--', ...
            'HandleVisibility','off');

    end

    grid on

    yticks([1 2 3 4])
    yticklabels({ ...
        'Balanced', ...
        'Comfort', ...
        'Low Force', ...
        'Protection'});

    ylim([0.5 4.5])
    xlim([0 road_length])

    ylabel('Selected Controller')

    title( ...
        'Model 3A Manual vs Model 3B Response-Based Controller Selection')

    legend('Location','northwest')

    set(gca, ...
        'FontWeight','bold', ...
        'FontSize',11)


    % =====================================================
    % BOTTOM: COMPOSITE ROAD
    % =====================================================

    ax2 = subplot(2,1,2);

    plot( ...
        sim3B.road_position, ...
        1000*sim3B.road, ...
        'LineWidth',1.3)

    hold on

    for i = 1:length(segment_edges)

        xline( ...
            segment_edges(i), ...
            '--', ...
            'HandleVisibility','off');

    end

    grid on

    xlim([0 road_length])

    ylabel('Road Height (mm)')
    xlabel('Road Position x (m)')

    title('Composite Benchmark Road')

    set(gca, ...
        'FontWeight','bold', ...
        'FontSize',11)


    % =====================================================
    % ALIGN BOTH PANELS
    % =====================================================

    linkaxes([ax1 ax2],'x')

end

%% 9. KEY FIGURE 2 - NOMINAL BUMP TRANSIENT

zoom_start = 57;
zoom_end = 62;

idx3A = sim3A.road_position >= zoom_start & ...
        sim3A.road_position <= zoom_end;

idx3B = sim3B.road_position >= zoom_start & ...
        sim3B.road_position <= zoom_end;

if show_figures

    figure('Name','Nominal Bump Response')

    subplot(5,1,1)
    plot(sim3B.road_position(idx3B), ...
        1000*sim3B.road(idx3B),'LineWidth',1.2)
    grid on
    ylabel('Road (mm)')
    title('Nominal Bump - Model 3A vs Model 3B')
    xline(58,'--','Bump Start','HandleVisibility','off')
    xline(59.2,'--','Bump End','HandleVisibility','off')
    xlim([zoom_start zoom_end])

    subplot(5,1,2)
    stairs(sim3A.road_position(idx3A),sim3A.mode(idx3A),'LineWidth',1.2)
    hold on
    stairs(sim3B.road_position(idx3B),sim3B.mode(idx3B),'LineWidth',1.3)
    grid on
    yticks([1 2 3 4])
    yticklabels({'Balanced','Comfort','Low Force','Protection'})
    ylabel('Mode')
    legend('3A','3B','Location','best')
    ylim([0.5 4.5])
    xlim([zoom_start zoom_end])

    subplot(5,1,3)
    plot(sim3A.road_position(idx3A),sim3A.BodyAccel(idx3A),'LineWidth',1.1)
    hold on
    plot(sim3B.road_position(idx3B),sim3B.BodyAccel(idx3B),'LineWidth',1.1)
    grid on
    ylabel('Body Accel.')
    xlim([zoom_start zoom_end])

    subplot(5,1,4)
    plot(sim3A.road_position(idx3A),1000*sim3A.SuspTravel(idx3A),'LineWidth',1.1)
    hold on
    plot(sim3B.road_position(idx3B),1000*sim3B.SuspTravel(idx3B),'LineWidth',1.1)
    grid on
    ylabel('Travel (mm)')
    xlim([zoom_start zoom_end])

    subplot(5,1,5)
    plot(sim3A.road_position(idx3A),sim3A.Fa(idx3A),'LineWidth',1.1)
    hold on
    plot(sim3B.road_position(idx3B),sim3B.Fa(idx3B),'LineWidth',1.1)
    grid on
    ylabel('Force (N)')
    xlabel('Road Position x (m)')
    xlim([zoom_start zoom_end])

end


%% 10. ROBUSTNESS TEST - BUMP HEIGHT x VEHICLE SPEED
%
% IMPORTANT:
% No controller gains, thresholds, or hold times are changed here.
% The same Model 3B scheduler function is reused for every test.

robust_speeds = [5 10 15];                    % [m/s]
robust_bump_heights = [0.015 0.030 0.045];     % [m]

robust_bump_length = 1.2;                      % [m]
robust_bump_start = 8.0;                       % [m]
robust_road_length = 30.0;                     % [m]

nSpeed = length(robust_speeds);
nHeight = length(robust_bump_heights);
nTests = nSpeed*nHeight;

Speed_mps = zeros(nTests,1);
BumpHeight_mm = zeros(nTests,1);
Accel_Peak_robust = zeros(nTests,1);
Travel_Peak_mm_robust = zeros(nTests,1);
TireDef_Peak_mm_robust = zeros(nTests,1);
Force_Peak_robust = zeros(nTests,1);
Protection_Time_ms = zeros(nTests,1);
Comfort_Time_ms = zeros(nTests,1);

% Diagnostics are stored but not printed by default.
First_Protection_x_m = NaN(nTests,1);
First_Comfort_x_m = NaN(nTests,1);
Max_RelVelocity = zeros(nTests,1);
Transient_Start_Count = zeros(nTests,1);

row = 0;

for iV = 1:nSpeed

    for iH = 1:nHeight

        row = row + 1;

        v_test = robust_speeds(iV);
        h_test = robust_bump_heights(iH);

        zr_test = @(x) isolatedBumpRoad( ...
            x, ...
            h_test, ...
            robust_bump_length, ...
            robust_bump_start);

        sim_test = simulateResponseScheduler( ...
            v_test, ...
            robust_road_length, ...
            zr_test, ...
            Ts, ...
            x0, ...
            A, B, E, ...
            K_bank, ...
            ms, mu, ks, cs, kt, ...
            window_samples, ...
            fast_window_samples, ...
            protection_hold_samples, ...
            shock_hold_samples, ...
            Body_Low_Threshold, ...
            Travel_Low_Threshold, ...
            Wheel_LowForce_Max, ...
            Wheel_Comfort_Max, ...
            Wheel_High_Threshold, ...
            Travel_Comfort_Min, ...
            Wheel_Fast_Protection_Threshold, ...
            Shock_RelVel_Threshold, ...
            Shock_Confirm_RelVel_Threshold, ...
            Shock_Confirm_Samples, ...
            Fmax_locked, ...
            Fdotmax_locked, ...
            noise_free, ...
            SensorDelaySamples_locked, ...
            SensorUpdateSamples_locked);

        % Evaluate from 0.2 s before bump entry to 1.0 s after exit.
        t_bump_start = robust_bump_start/v_test;
        t_bump_end = (robust_bump_start+robust_bump_length)/v_test;

        idx_eval = ...
            sim_test.t >= max(0,t_bump_start-0.2) & ...
            sim_test.t <= min(sim_test.t(end),t_bump_end+1.0);

        Speed_mps(row) = v_test;
        BumpHeight_mm(row) = 1000*h_test;

        Accel_Peak_robust(row) = ...
            max(abs(sim_test.BodyAccel(idx_eval)));

        Travel_Peak_mm_robust(row) = ...
            1000*max(abs(sim_test.SuspTravel(idx_eval)));

        TireDef_Peak_mm_robust(row) = ...
            1000*max(abs(sim_test.TireDef(idx_eval)));

        Force_Peak_robust(row) = ...
            max(abs(sim_test.Fa(idx_eval)));

        Protection_Time_ms(row) = ...
            1000*Ts*sum(sim_test.mode(idx_eval)==4);

        Comfort_Time_ms(row) = ...
            1000*Ts*sum(sim_test.mode(idx_eval)==2);

        Max_RelVelocity(row) = ...
            max(abs(sim_test.RelVelocity(idx_eval)));

        Transient_Start_Count(row) = ...
            sim_test.transient_start_count;

        idx_after_bump = find(sim_test.road_position >= robust_bump_start);

        local_idx = find(sim_test.mode(idx_after_bump)==4,1,'first');
        if ~isempty(local_idx)
            First_Protection_x_m(row) = ...
                sim_test.road_position(idx_after_bump(local_idx));
        end

        local_idx = find(sim_test.mode(idx_after_bump)==2,1,'first');
        if ~isempty(local_idx)
            First_Comfort_x_m(row) = ...
                sim_test.road_position(idx_after_bump(local_idx));
        end

    end

end

% Compact command-window table.
RobustnessSummary = table( ...
    Speed_mps, ...
    BumpHeight_mm, ...
    Accel_Peak_robust, ...
    Travel_Peak_mm_robust, ...
    TireDef_Peak_mm_robust, ...
    Force_Peak_robust, ...
    Protection_Time_ms, ...
    Comfort_Time_ms, ...
    'VariableNames', { ...
    'Speed_mps', ...
    'BumpHeight_mm', ...
    'Accel_Peak', ...
    'Travel_Peak_mm', ...
    'TireDef_Peak_mm', ...
    'Force_Peak', ...
    'Protection_Time_ms', ...
    'Comfort_Time_ms'});

% Detailed diagnostics remain available in the workspace if needed.
RobustnessDiagnostics = table( ...
    Speed_mps, ...
    BumpHeight_mm, ...
    First_Protection_x_m, ...
    First_Comfort_x_m, ...
    Max_RelVelocity, ...
    Transient_Start_Count);

% Bump robustness was already validated.
% Keep RobustnessSummary in the workspace, but do not print it every run.


%% 11. ROBUSTNESS TEST 2 - UNSEEN MIXED / IRREGULAR ROADS
%
% Purpose:
% Test whether Model 3B generalizes to road profiles that were NOT used
% when the scheduler thresholds were developed.
%
% Three deterministic multi-sine roads are used:
%   1 = Mixed Mild
%   2 = Mixed Medium
%   3 = Mixed Rough
%
% Each road combines several wavelengths and phase offsets.
% No controller gains, thresholds, windows, or hold times are retuned.
%
% Model 3B is compared against the fixed Balanced controller.
% Negative percentage change = Model 3B is better for that metric.

irregular_speeds = [5 10 15];     % [m/s]

irregular_profile_names = [ ...
    "Mixed Mild"
    "Mixed Medium"
    "Mixed Rough"
    ];

irregular_road_length = 60;        % [m]

% Road disturbance is smoothly ramped in/out so edge discontinuities
% do not dominate the comparison.
irregular_road_start = 5;          % [m]
irregular_road_end = 55;           % [m]
irregular_ramp_length = 5;         % [m]

% Evaluate only the full-amplitude interior region.
irregular_eval_start = 10;         % [m]
irregular_eval_end = 50;           % [m]

nIrregSpeed = length(irregular_speeds);
nIrregProfile = length(irregular_profile_names);
nIrregTests = nIrregSpeed*nIrregProfile;

RoadProfile = strings(nIrregTests,1);
IrregSpeed_mps = zeros(nIrregTests,1);
RoadRMS_mm = zeros(nIrregTests,1);

AccelRMS_3B_irreg = zeros(nIrregTests,1);
AccelRMS_Bal_irreg = zeros(nIrregTests,1);

AccelPeak_3B_irreg = zeros(nIrregTests,1);
AccelPeak_Bal_irreg = zeros(nIrregTests,1);

TravelPeak_3B_mm_irreg = zeros(nIrregTests,1);
TravelPeak_Bal_mm_irreg = zeros(nIrregTests,1);

TirePeak_3B_mm_irreg = zeros(nIrregTests,1);
TirePeak_Bal_mm_irreg = zeros(nIrregTests,1);

ForceRMS_3B_irreg = zeros(nIrregTests,1);
ForceRMS_Bal_irreg = zeros(nIrregTests,1);

ForcePeak_3B_irreg = zeros(nIrregTests,1);
ForcePeak_Bal_irreg = zeros(nIrregTests,1);

Balanced_pct_irreg = zeros(nIrregTests,1);
Comfort_pct_irreg = zeros(nIrregTests,1);
LowForce_pct_irreg = zeros(nIrregTests,1);
Protection_pct_irreg = zeros(nIrregTests,1);

TransientCount_irreg = zeros(nIrregTests,1);

row = 0;

for iV = 1:nIrregSpeed

    for iP = 1:nIrregProfile

        row = row + 1;

        v_test = irregular_speeds(iV);
        profile_id = iP;

        zr_test = @(x) mixedRoadProfile( ...
            x, ...
            profile_id, ...
            irregular_road_start, ...
            irregular_road_end, ...
            irregular_ramp_length);

        % Response-based Model 3B
        sim_irreg_3B = simulateResponseScheduler( ...
            v_test, ...
            irregular_road_length, ...
            zr_test, ...
            Ts, ...
            x0, ...
            A, B, E, ...
            K_bank, ...
            ms, mu, ks, cs, kt, ...
            window_samples, ...
            fast_window_samples, ...
            protection_hold_samples, ...
            shock_hold_samples, ...
            Body_Low_Threshold, ...
            Travel_Low_Threshold, ...
            Wheel_LowForce_Max, ...
            Wheel_Comfort_Max, ...
            Wheel_High_Threshold, ...
            Travel_Comfort_Min, ...
            Wheel_Fast_Protection_Threshold, ...
            Shock_RelVel_Threshold, ...
            Shock_Confirm_RelVel_Threshold, ...
            Shock_Confirm_Samples, ...
            Fmax_locked, ...
            Fdotmax_locked, ...
            noise_free, ...
            SensorDelaySamples_locked, ...
            SensorUpdateSamples_locked);

        % Fixed Balanced baseline
        sim_irreg_bal = simulateFixedController( ...
            v_test, ...
            irregular_road_length, ...
            zr_test, ...
            Ts, ...
            x0, ...
            A, B, E, ...
            K_bank(1,:), ...
            ms, mu, ks, cs, kt, ...
            Fmax_locked, ...
            Fdotmax_locked);

        idx_3B = ...
            sim_irreg_3B.road_position >= irregular_eval_start & ...
            sim_irreg_3B.road_position <= irregular_eval_end;

        idx_bal = ...
            sim_irreg_bal.road_position >= irregular_eval_start & ...
            sim_irreg_bal.road_position <= irregular_eval_end;

        RoadProfile(row) = irregular_profile_names(iP);
        IrregSpeed_mps(row) = v_test;

        RoadRMS_mm(row) = ...
            1000*rms(sim_irreg_3B.road(idx_3B));

        AccelRMS_3B_irreg(row) = ...
            rms(sim_irreg_3B.BodyAccel(idx_3B));

        AccelRMS_Bal_irreg(row) = ...
            rms(sim_irreg_bal.BodyAccel(idx_bal));

        AccelPeak_3B_irreg(row) = ...
            max(abs(sim_irreg_3B.BodyAccel(idx_3B)));

        AccelPeak_Bal_irreg(row) = ...
            max(abs(sim_irreg_bal.BodyAccel(idx_bal)));

        TravelPeak_3B_mm_irreg(row) = ...
            1000*max(abs(sim_irreg_3B.SuspTravel(idx_3B)));

        TravelPeak_Bal_mm_irreg(row) = ...
            1000*max(abs(sim_irreg_bal.SuspTravel(idx_bal)));

        TirePeak_3B_mm_irreg(row) = ...
            1000*max(abs(sim_irreg_3B.TireDef(idx_3B)));

        TirePeak_Bal_mm_irreg(row) = ...
            1000*max(abs(sim_irreg_bal.TireDef(idx_bal)));

        ForceRMS_3B_irreg(row) = ...
            rms(sim_irreg_3B.Fa(idx_3B));

        ForceRMS_Bal_irreg(row) = ...
            rms(sim_irreg_bal.Fa(idx_bal));

        ForcePeak_3B_irreg(row) = ...
            max(abs(sim_irreg_3B.Fa(idx_3B)));

        ForcePeak_Bal_irreg(row) = ...
            max(abs(sim_irreg_bal.Fa(idx_bal)));

        n_eval = sum(idx_3B);

        Balanced_pct_irreg(row) = ...
            100*sum(sim_irreg_3B.mode(idx_3B)==1)/n_eval;

        Comfort_pct_irreg(row) = ...
            100*sum(sim_irreg_3B.mode(idx_3B)==2)/n_eval;

        LowForce_pct_irreg(row) = ...
            100*sum(sim_irreg_3B.mode(idx_3B)==3)/n_eval;

        Protection_pct_irreg(row) = ...
            100*sum(sim_irreg_3B.mode(idx_3B)==4)/n_eval;

        TransientCount_irreg(row) = ...
            sim_irreg_3B.transient_start_count;

    end

end

%% UNSEEN MIXED ROAD PROFILES - REPORT FIGURE

if show_figures

    x_road_plot = linspace(0,irregular_road_length,5000);

    figure('Name','Unseen Mixed Road Profiles', ...
        'Position',[100 100 1000 700])

    for iP = 1:3

        zr_plot = mixedRoadProfile( ...
            x_road_plot, ...
            iP, ...
            irregular_road_start, ...
            irregular_road_end, ...
            irregular_ramp_length);

        subplot(3,1,iP)

        plot(x_road_plot,1000*zr_plot,'LineWidth',1.3)
        grid on

        ylabel('Road (mm)')
        xlim([0 irregular_road_length])

        if iP == 1
            title('Mixed Mild')
        elseif iP == 2
            title('Mixed Medium')
        else
            title('Mixed Rough')
        end

        set(gca,'FontWeight','bold','FontSize',11)

    end

    xlabel('Road Position x (m)','FontWeight','bold')

end


% Percentage change relative to the fixed Balanced baseline.
AccelRMS_vs_Bal_pct = ...
    100*(AccelRMS_3B_irreg./AccelRMS_Bal_irreg - 1);

TravelPeak_vs_Bal_pct = ...
    100*(TravelPeak_3B_mm_irreg./TravelPeak_Bal_mm_irreg - 1);

TirePeak_vs_Bal_pct = ...
    100*(TirePeak_3B_mm_irreg./TirePeak_Bal_mm_irreg - 1);

ForceRMS_vs_Bal_pct = ...
    100*(ForceRMS_3B_irreg./ForceRMS_Bal_irreg - 1);


% Compact printed summary.
IrregularRobustnessSummary = table( ...
    RoadProfile, ...
    IrregSpeed_mps, ...
    AccelRMS_vs_Bal_pct, ...
    TravelPeak_vs_Bal_pct, ...
    TirePeak_vs_Bal_pct, ...
    ForceRMS_vs_Bal_pct, ...
    Balanced_pct_irreg, ...
    Protection_pct_irreg, ...
    Comfort_pct_irreg, ...
    LowForce_pct_irreg, ...
    'VariableNames', { ...
    'RoadProfile', ...
    'Speed_mps', ...
    'AccelRMS_vs_Bal_pct', ...
    'TravelPeak_vs_Bal_pct', ...
    'TirePeak_vs_Bal_pct', ...
    'ForceRMS_vs_Bal_pct', ...
    'Balanced_pct', ...
    'Protection_pct', ...
    'Comfort_pct', ...
    'LowForce_pct'});


% Raw values and extra diagnostics stay in the workspace.
IrregularDiagnostics = table( ...
    RoadProfile, ...
    IrregSpeed_mps, ...
    RoadRMS_mm, ...
    AccelRMS_3B_irreg, ...
    AccelRMS_Bal_irreg, ...
    AccelPeak_3B_irreg, ...
    AccelPeak_Bal_irreg, ...
    TravelPeak_3B_mm_irreg, ...
    TravelPeak_Bal_mm_irreg, ...
    TirePeak_3B_mm_irreg, ...
    TirePeak_Bal_mm_irreg, ...
    ForceRMS_3B_irreg, ...
    ForceRMS_Bal_irreg, ...
    ForcePeak_3B_irreg, ...
    ForcePeak_Bal_irreg, ...
    Balanced_pct_irreg, ...
    Comfort_pct_irreg, ...
    LowForce_pct_irreg, ...
    Protection_pct_irreg, ...
    TransientCount_irreg);

if show_tables
    disp(' ')
    disp('--- ROBUSTNESS: UNSEEN MIXED ROADS ---')
    disp(IrregularRobustnessSummary)
end



%% 11.1 KEY FIGURE 3 - IRREGULAR-ROAD ROBUSTNESS SUMMARY

if show_figures

    figure('Name','Irregular Road Robustness Summary')

    subplot(2,2,1)
    hold on
    grid on

    for iP = 1:nIrregProfile

        idx = RoadProfile == irregular_profile_names(iP);

        plot( ...
            IrregSpeed_mps(idx), ...
            AccelRMS_vs_Bal_pct(idx), ...
            '-o', ...
            'LineWidth',1.2, ...
            'DisplayName',irregular_profile_names(iP));

    end

    yline(0,'--','Balanced reference', ...
        'HandleVisibility','off')

    ylabel('\Delta Accel RMS vs Balanced (%)')
    title('Unseen Mixed Roads')
    legend('Location','best')


    subplot(2,2,2)
    hold on
    grid on

    for iP = 1:nIrregProfile

        idx = RoadProfile == irregular_profile_names(iP);

        plot( ...
            IrregSpeed_mps(idx), ...
            TravelPeak_vs_Bal_pct(idx), ...
            '-o', ...
            'LineWidth',1.2);

    end

    yline(0,'--','HandleVisibility','off')

    ylabel('\Delta Travel Peak (%)')


    subplot(2,2,3)
    hold on
    grid on

    for iP = 1:nIrregProfile

        idx = RoadProfile == irregular_profile_names(iP);

        plot( ...
            IrregSpeed_mps(idx), ...
            ForceRMS_vs_Bal_pct(idx), ...
            '-o', ...
            'LineWidth',1.2);

    end

    yline(0,'--','HandleVisibility','off')

    xlabel('Vehicle Speed (m/s)')
    ylabel('\Delta Force RMS (%)')


    subplot(2,2,4)
    hold on
    grid on

    for iP = 1:nIrregProfile

        idx = RoadProfile == irregular_profile_names(iP);

        plot( ...
            IrregSpeed_mps(idx), ...
            Protection_pct_irreg(idx), ...
            '-o', ...
            'LineWidth',1.2);

    end

    xlabel('Vehicle Speed (m/s)')
    ylabel('Protection Usage (%)')

end






%% 12. FINAL COMBINED REALISM VALIDATION
%
% This is the final V1 realism test.
%
% EVERYTHING BELOW IS APPLIED AT THE SAME TIME:
%
%   Controller / scheduler:
%       Model 3B V1 logic LOCKED
%
%   Actuator:
%       |Fa|     <= 750 N
%       |dFa/dt| <= 50,000 N/s
%
%   Measurement timing:
%       fs = 500 Hz
%       sensor transport delay = 4 ms
%
%   Shock confirmation:
%       |v_rel,meas| > 0.80 m/s
%       for 3 consecutive NEW sensor samples
%
%   Measurement noise:
%       Ideal / Low / Moderate / High
%
% No gains, thresholds, timers, or actuator parameters are retuned.
%
% Reference:
% For each road scenario, percentage changes are measured against the
% same locked V1 plant/controller with:
%
%       zero sensor delay
%       500 Hz sampling
%       ideal/noiseless measurements
%
% Therefore the table reports the TOTAL effect of the locked timing
% realism + measurement noise.

combined_noise_names = [ ...
    "Ideal"
    "Low"
    "Moderate"
    "High"
    ];

combined_body_std = [0; 0.02; 0.05; 0.10];             % [m/s^2]
combined_wheel_std = [0; 0.10; 0.30; 0.80];            % [m/s^2]
combined_travel_std = [0; 0.00005; 0.00010; 0.00025]; % [m]
combined_relvel_std = [0; 0.005; 0.010; 0.030];        % [m/s]

combined_scenario_names = [ ...
    "Nominal Composite"
    "Mixed Medium - 10 m/s"
    "Mixed Rough - 15 m/s"
    ];

nCombinedNoise = length(combined_noise_names);
nCombinedScenario = length(combined_scenario_names);
nCombined = nCombinedNoise*nCombinedScenario;

CombinedScenario = strings(nCombined,1);
CombinedNoiseLevel = strings(nCombined,1);

CombinedModeMismatch_pct = zeros(nCombined,1);

CombinedAccelRMS = zeros(nCombined,1);
CombinedAccelPeak = zeros(nCombined,1);
CombinedTravelPeak_mm = zeros(nCombined,1);
CombinedTirePeak_mm = zeros(nCombined,1);
CombinedForceRMS = zeros(nCombined,1);

CombinedAccelRMS_change_pct = zeros(nCombined,1);
CombinedAccelPeak_change_pct = zeros(nCombined,1);
CombinedTravelPeak_change_pct = zeros(nCombined,1);
CombinedTirePeak_change_pct = zeros(nCombined,1);
CombinedForceRMS_change_pct = zeros(nCombined,1);

CombinedProtection_pct = zeros(nCombined,1);
CombinedModeChangeCount = zeros(nCombined,1);
CombinedTransientCount = zeros(nCombined,1);

% Nominal-event diagnostics are stored separately below.
NominalNoiseLevel = combined_noise_names;

NominalBumpConfirmed = false(nCombinedNoise,1);
NominalBumpConfirmPosition_m = NaN(nCombinedNoise,1);

NominalWheelHopConfirmed = false(nCombinedNoise,1);
NominalWheelHopConfirmPosition_m = NaN(nCombinedNoise,1);

NominalTransientCount = zeros(nCombinedNoise,1);

row = 0;

for iS = 1:nCombinedScenario

    % -----------------------------------------------------
    % Scenario
    % -----------------------------------------------------

    if iS == 1

        v_comb = v;
        road_length_comb = road_length;
        zr_comb = zr_composite;

        eval_start_comb = 0;
        eval_end_comb = road_length;

    elseif iS == 2

        v_comb = 10;
        road_length_comb = irregular_road_length;

        zr_comb = @(x) mixedRoadProfile( ...
            x, ...
            2, ...
            irregular_road_start, ...
            irregular_road_end, ...
            irregular_ramp_length);

        eval_start_comb = irregular_eval_start;
        eval_end_comb = irregular_eval_end;

    else

        v_comb = 15;
        road_length_comb = irregular_road_length;

        zr_comb = @(x) mixedRoadProfile( ...
            x, ...
            3, ...
            irregular_road_start, ...
            irregular_road_end, ...
            irregular_ramp_length);

        eval_start_comb = irregular_eval_start;
        eval_end_comb = irregular_eval_end;

    end


    % -----------------------------------------------------
    % Zero-delay / ideal-measurement reference for this road.
    % Same locked actuator and scheduler.
    % -----------------------------------------------------

    rng(3000+iS);

    sim_comb_ref = simulateResponseScheduler( ...
        v_comb, ...
        road_length_comb, ...
        zr_comb, ...
        Ts, ...
        x0, ...
        A, B, E, ...
        K_bank, ...
        ms, mu, ks, cs, kt, ...
        window_samples, ...
        fast_window_samples, ...
        protection_hold_samples, ...
        shock_hold_samples, ...
        Body_Low_Threshold, ...
        Travel_Low_Threshold, ...
        Wheel_LowForce_Max, ...
        Wheel_Comfort_Max, ...
        Wheel_High_Threshold, ...
        Travel_Comfort_Min, ...
        Wheel_Fast_Protection_Threshold, ...
        Shock_RelVel_Threshold, ...
        Shock_Confirm_RelVel_Threshold, ...
        Shock_Confirm_Samples, ...
        Fmax_locked, ...
        Fdotmax_locked, ...
        noise_free, ...
        0, ...
        SensorUpdateSamples_locked);

    idx_ref_comb = ...
        sim_comb_ref.road_position >= eval_start_comb & ...
        sim_comb_ref.road_position <= eval_end_comb;

    ref_mode_comb = sim_comb_ref.mode(idx_ref_comb);

    ref_accel_rms = ...
        rms(sim_comb_ref.BodyAccel(idx_ref_comb));

    ref_accel_peak = ...
        max(abs(sim_comb_ref.BodyAccel(idx_ref_comb)));

    ref_travel_peak = ...
        1000*max(abs(sim_comb_ref.SuspTravel(idx_ref_comb)));

    ref_tire_peak = ...
        1000*max(abs(sim_comb_ref.TireDef(idx_ref_comb)));

    ref_force_rms = ...
        rms(sim_comb_ref.Fa(idx_ref_comb));


    % -----------------------------------------------------
    % Locked timing + noise levels
    % -----------------------------------------------------

    for iN = 1:nCombinedNoise

        row = row + 1;

        combined_noise_cfg = struct( ...
            'bodyAccelStd',combined_body_std(iN), ...
            'wheelAccelStd',combined_wheel_std(iN), ...
            'travelStd',combined_travel_std(iN), ...
            'relVelStd',combined_relvel_std(iN));

        % Same random sequence within a scenario for paired comparison.
        rng(3000+iS);

        sim_comb = simulateResponseScheduler( ...
            v_comb, ...
            road_length_comb, ...
            zr_comb, ...
            Ts, ...
            x0, ...
            A, B, E, ...
            K_bank, ...
            ms, mu, ks, cs, kt, ...
            window_samples, ...
            fast_window_samples, ...
            protection_hold_samples, ...
            shock_hold_samples, ...
            Body_Low_Threshold, ...
            Travel_Low_Threshold, ...
            Wheel_LowForce_Max, ...
            Wheel_Comfort_Max, ...
            Wheel_High_Threshold, ...
            Travel_Comfort_Min, ...
            Wheel_Fast_Protection_Threshold, ...
            Shock_RelVel_Threshold, ...
            Shock_Confirm_RelVel_Threshold, ...
            Shock_Confirm_Samples, ...
            Fmax_locked, ...
            Fdotmax_locked, ...
            combined_noise_cfg, ...
            SensorDelaySamples_locked, ...
            SensorUpdateSamples_locked);

        idx_comb = ...
            sim_comb.road_position >= eval_start_comb & ...
            sim_comb.road_position <= eval_end_comb;

        mode_comb = sim_comb.mode(idx_comb);

        CombinedScenario(row) = combined_scenario_names(iS);
        CombinedNoiseLevel(row) = combined_noise_names(iN);

        CombinedModeMismatch_pct(row) = ...
            100*mean(mode_comb ~= ref_mode_comb);

        CombinedAccelRMS(row) = ...
            rms(sim_comb.BodyAccel(idx_comb));

        CombinedAccelPeak(row) = ...
            max(abs(sim_comb.BodyAccel(idx_comb)));

        CombinedTravelPeak_mm(row) = ...
            1000*max(abs(sim_comb.SuspTravel(idx_comb)));

        CombinedTirePeak_mm(row) = ...
            1000*max(abs(sim_comb.TireDef(idx_comb)));

        CombinedForceRMS(row) = ...
            rms(sim_comb.Fa(idx_comb));

        CombinedAccelRMS_change_pct(row) = ...
            100*(CombinedAccelRMS(row)/ref_accel_rms - 1);

        CombinedAccelPeak_change_pct(row) = ...
            100*(CombinedAccelPeak(row)/ref_accel_peak - 1);

        CombinedTravelPeak_change_pct(row) = ...
            100*(CombinedTravelPeak_mm(row)/ref_travel_peak - 1);

        CombinedTirePeak_change_pct(row) = ...
            100*(CombinedTirePeak_mm(row)/ref_tire_peak - 1);

        CombinedForceRMS_change_pct(row) = ...
            100*(CombinedForceRMS(row)/ref_force_rms - 1);

        CombinedProtection_pct(row) = ...
            100*mean(mode_comb == 4);

        CombinedModeChangeCount(row) = ...
            sum(diff(mode_comb)~=0);

        CombinedTransientCount(row) = ...
            sim_comb.transient_start_count;


        % -------------------------------------------------
        % Nominal bump / wheel-hop event validation
        % -------------------------------------------------

        if iS == 1

            % Broad non-overlapping windows around the known events.
            idx_bump_event = ...
                sim_comb.road_position >= 58.0 & ...
                sim_comb.road_position < 63.5;

            idx_wheel_event = ...
                sim_comb.road_position >= 64.0 & ...
                sim_comb.road_position <= 66.5;

            bump_event_indices = find(idx_bump_event);
            wheel_event_indices = find(idx_wheel_event);

            bump_confirm_local = find( ...
                sim_comb.ShockConfirmed(idx_bump_event), ...
                1, ...
                'first');

            wheel_confirm_local = find( ...
                sim_comb.ShockConfirmed(idx_wheel_event), ...
                1, ...
                'first');

            if ~isempty(bump_confirm_local)

                NominalBumpConfirmed(iN) = true;

                NominalBumpConfirmPosition_m(iN) = ...
                    sim_comb.road_position( ...
                    bump_event_indices(bump_confirm_local));

            end

            if ~isempty(wheel_confirm_local)

                NominalWheelHopConfirmed(iN) = true;

                NominalWheelHopConfirmPosition_m(iN) = ...
                    sim_comb.road_position( ...
                    wheel_event_indices(wheel_confirm_local));

            end

            NominalTransientCount(iN) = ...
                sim_comb.transient_start_count;

        end

    end

end


CombinedRealismSummary = table( ...
    CombinedScenario, ...
    CombinedNoiseLevel, ...
    CombinedModeMismatch_pct, ...
    CombinedAccelRMS_change_pct, ...
    CombinedAccelPeak_change_pct, ...
    CombinedTravelPeak_change_pct, ...
    CombinedTirePeak_change_pct, ...
    CombinedForceRMS_change_pct, ...
    CombinedProtection_pct, ...
    CombinedModeChangeCount, ...
    CombinedTransientCount, ...
    'VariableNames', { ...
    'Scenario', ...
    'NoiseLevel', ...
    'ModeMismatch_pct', ...
    'AccelRMS_change_pct', ...
    'AccelPeak_change_pct', ...
    'TravelPeak_change_pct', ...
    'TirePeak_change_pct', ...
    'ForceRMS_change_pct', ...
    'Protection_pct', ...
    'ModeChangeCount', ...
    'TransientCount'});


CombinedNominalEventValidation = table( ...
    NominalNoiseLevel, ...
    NominalBumpConfirmed, ...
    NominalBumpConfirmPosition_m, ...
    NominalWheelHopConfirmed, ...
    NominalWheelHopConfirmPosition_m, ...
    NominalTransientCount, ...
    'VariableNames', { ...
    'NoiseLevel', ...
    'BumpConfirmed', ...
    'BumpConfirmPosition_m', ...
    'WheelHopConfirmed', ...
    'WheelHopConfirmPosition_m', ...
    'TransientCount'});


% Raw combined values remain available in the workspace.
CombinedRealismDiagnostics = table( ...
    CombinedScenario, ...
    CombinedNoiseLevel, ...
    CombinedAccelRMS, ...
    CombinedAccelPeak, ...
    CombinedTravelPeak_mm, ...
    CombinedTirePeak_mm, ...
    CombinedForceRMS, ...
    CombinedProtection_pct, ...
    CombinedModeChangeCount, ...
    CombinedTransientCount);


if show_tables

    disp(' ')
    disp('--- FINAL COMBINED REALISM VALIDATION ---')
    disp(CombinedRealismSummary)

    disp(' ')
    disp('--- COMBINED NOMINAL EVENT VALIDATION ---')
    disp(CombinedNominalEventValidation)

end




%% 7.X NOMINAL MODEL 2 vs MODEL 3B COMPARISON

% Fixed Balanced controller under the SAME actuator constraints
simBalancedNominal = simulateFixedController( ...
    v, ...
    road_length, ...
    zr_composite, ...
    Ts, ...
    x0, ...
    A, B, E, ...
    K_bank(1,:), ...
    ms, mu, ks, cs, kt, ...
    Fmax_locked, ...
    Fdotmax_locked);


%% Global metrics

ModelNames = [ ...
    "Fixed Balanced LQR"
    "Model 3B Response-Based"
    ];

Accel_RMS_compare = [ ...
    rms(simBalancedNominal.BodyAccel)
    rms(sim3B.BodyAccel)
    ];

Accel_Peak_compare = [ ...
    max(abs(simBalancedNominal.BodyAccel))
    max(abs(sim3B.BodyAccel))
    ];

Travel_Peak_compare_mm = 1000*[ ...
    max(abs(simBalancedNominal.SuspTravel))
    max(abs(sim3B.SuspTravel))
    ];

Tire_Peak_compare_mm = 1000*[ ...
    max(abs(simBalancedNominal.TireDef))
    max(abs(sim3B.TireDef))
    ];

Force_RMS_compare = [ ...
    rms(simBalancedNominal.Fa)
    rms(sim3B.Fa)
    ];

Force_Peak_compare = [ ...
    max(abs(simBalancedNominal.Fa))
    max(abs(sim3B.Fa))
    ];

Nominal3BComparison = table( ...
    ModelNames, ...
    Accel_RMS_compare, ...
    Accel_Peak_compare, ...
    Travel_Peak_compare_mm, ...
    Tire_Peak_compare_mm, ...
    Force_RMS_compare, ...
    Force_Peak_compare, ...
    'VariableNames', { ...
    'Model', ...
    'Accel_RMS', ...
    'Accel_Peak', ...
    'Travel_Peak_mm', ...
    'TireDef_Peak_mm', ...
    'Force_RMS', ...
    'Force_Peak'});

disp(' ')
disp('--- NOMINAL FIXED LQR vs MODEL 3B ---')
disp(Nominal3BComparison)



%% LOCAL FUNCTIONS

% Normal scheduling philosophy:
%   Protection = sustained wheel-risk / fast transient safety response
%   Comfort    = body-dominated response with low wheel activity
%   Low Force  = high-confidence calm/economy mode only
%   Balanced   = mixed / ambiguous dynamics controller
%
function sim = simulateResponseScheduler( ...
    v, ...
    road_length, ...
    roadFcn, ...
    Ts, ...
    x0, ...
    A, B, E, ...
    K_bank, ...
    ms, mu, ks, cs, kt, ...
    window_samples, ...
    fast_window_samples, ...
    protection_hold_samples, ...
    shock_hold_samples, ...
    Body_Low_Threshold, ...
    Travel_Low_Threshold, ...
    Wheel_LowForce_Max, ...
    Wheel_Comfort_Max, ...
    Wheel_High_Threshold, ...
    Travel_Comfort_Min, ...
    Wheel_Fast_Protection_Threshold, ...
    Shock_RelVel_Threshold, ...
    Shock_Confirm_RelVel_Threshold, ...
    Shock_Confirm_Samples, ...
    varargin)

% Controller modes:
% 1 = Balanced
% 2 = Comfort
% 3 = Low Force
% 4 = Protection

% Optional physical actuator + sensor settings.
%
% varargin{1} = force magnitude limit Fmax [N]
% varargin{2} = force-rate limit Fdotmax [N/s]
% varargin{3} = measurement-noise configuration struct
% varargin{4} = sensor transport delay [base simulation samples]
% varargin{5} = sensor update interval [base simulation samples]
%
% Examples at Ts = 0.002 s:
%   sensorDelaySamples = 5  -> 10 ms latency
%   sensorUpdateSamples = 5 -> new measurement every 10 ms (100 Hz)
%
% Delayed/held measurements affect only the scheduler.
% Performance metrics remain based on the true plant response.

default_noise = struct( ...
    'bodyAccelStd',0, ...
    'wheelAccelStd',0, ...
    'travelStd',0, ...
    'relVelStd',0);

Fmax = Inf;
Fdotmax = Inf;
noiseCfg = default_noise;
sensorDelaySamples = 0;
sensorUpdateSamples = 1;

if length(varargin) >= 1
    Fmax = varargin{1};
end

if length(varargin) >= 2
    Fdotmax = varargin{2};
end

if length(varargin) >= 3
    noiseCfg = varargin{3};
end

if length(varargin) >= 4
    sensorDelaySamples = varargin{4};
end

if length(varargin) >= 5
    sensorUpdateSamples = varargin{5};
end

if ~(isscalar(Fmax) && Fmax > 0)
    error('Fmax must be a positive scalar or Inf.')
end

if ~(isscalar(Fdotmax) && Fdotmax > 0)
    error('Fdotmax must be a positive scalar or Inf.')
end

if ~(isscalar(sensorDelaySamples) && ...
        sensorDelaySamples >= 0 && ...
        sensorDelaySamples == round(sensorDelaySamples))
    error('sensorDelaySamples must be a nonnegative integer.')
end

if ~(isscalar(sensorUpdateSamples) && ...
        sensorUpdateSamples >= 1 && ...
        sensorUpdateSamples == round(sensorUpdateSamples))
    error('sensorUpdateSamples must be a positive integer.')
end

K_balanced = K_bank(1,:);

t_end = road_length/v;
t = (0:Ts:t_end)';
N = length(t);

x = zeros(N,4);
x(1,:) = x0';

FaCmd = zeros(N,1);
Fa = zeros(N,1);

Saturated = false(N,1);
RateLimited = false(N,1);

BodyAccel = zeros(N,1);
WheelAccel = zeros(N,1);
SuspTravel = zeros(N,1);
RelVelocity = zeros(N,1);

% Raw sensor outputs before transport delay / sample-and-hold.
BodyAccelRawMeas = zeros(N,1);
WheelAccelRawMeas = zeros(N,1);
SuspTravelRawMeas = zeros(N,1);
RelVelocityRawMeas = zeros(N,1);

% Measurements actually available to the scheduler after
% sensor delay and sensor update/sample-and-hold behavior.
BodyAccelMeas = zeros(N,1);
WheelAccelMeas = zeros(N,1);
SuspTravelMeas = zeros(N,1);
RelVelocityMeas = zeros(N,1);

MeasurementUpdated = false(N,1);

BodyRMS = zeros(N,1);
WheelRMS = zeros(N,1);
WheelRMSFast = zeros(N,1);
TravelRMS = zeros(N,1);

mode = ones(N,1);

% Event diagnostics. These do NOT affect controller logic.
ShockDetected = false(N,1);
ShockConfirmCandidate = false(N,1);
ShockConfirmed = false(N,1);
FastEventDetected = false(N,1);
TransientStarted = false(N,1);

current_mode = 1;
K_current = K_balanced;

% Actual actuator force carried between digital samples.
Fa_state = 0;

protection_counter = 0;
shock_counter = 0;

% Consecutive measured-relative-velocity samples above the
% dedicated shock-confirmation threshold.
shock_confirm_counter = 0;

transient_sequence_active = false;
shock_seen_in_transient = false;

% Entry-only event latch:
% after a transient starts, fast/shock detection cannot start another
% transient until BOTH event indicators have cleared.
transient_armed = true;
transient_start_count = 0;

for k = 1:N

    xk = x(k,:)';

    zs = xk(1);
    zs_dot = xk(2);
    zu = xk(3);
    zu_dot = xk(4);

    road_position_k = v*t(k);
    zr = roadFcn(road_position_k);

    % Current controller command.
    Fa_cmd_k = -K_current*xk;

    if isinf(Fdotmax)

        % Magnitude-limited V1 behavior.
        Fa_k = max(-Fmax,min(Fmax,Fa_cmd_k));

    else

        % Actual physical force reached by the rate-limited actuator
        % at the end of the previous sample interval.
        Fa_k = Fa_state;

    end

    body_accel = ...
        (-ks*zs ...
        -cs*zs_dot ...
        +ks*zu ...
        +cs*zu_dot ...
        +Fa_k)/ms;

    wheel_accel = ...
        (ks*zs ...
        +cs*zs_dot ...
        -(ks+kt)*zu ...
        -cs*zu_dot ...
        -Fa_k ...
        +kt*zr)/mu;

    susp_travel = zs - zu;
    rel_velocity = zs_dot - zu_dot;

    FaCmd(k) = Fa_cmd_k;
    Fa(k) = Fa_k;
    Saturated(k) = abs(Fa_cmd_k) > Fmax;

    BodyAccel(k) = body_accel;
    WheelAccel(k) = wheel_accel;
    SuspTravel(k) = susp_travel;
    RelVelocity(k) = rel_velocity;

    % -----------------------------------------------------
    % SENSOR PIPELINE
    %
    % true response
    %      -> measurement noise
    %      -> transport delay
    %      -> finite sensor update rate / zero-order hold
    %      -> scheduler
    % -----------------------------------------------------

    BodyAccelRawMeas(k) = ...
        body_accel + noiseCfg.bodyAccelStd*randn;

    WheelAccelRawMeas(k) = ...
        wheel_accel + noiseCfg.wheelAccelStd*randn;

    SuspTravelRawMeas(k) = ...
        susp_travel + noiseCfg.travelStd*randn;

    RelVelocityRawMeas(k) = ...
        rel_velocity + noiseCfg.relVelStd*randn;

    measurement_update_now = ...
        (k == 1) || mod(k-1,sensorUpdateSamples) == 0;

    if measurement_update_now

        delayed_idx = ...
            max(1,k-sensorDelaySamples);

        BodyAccelMeas(k) = ...
            BodyAccelRawMeas(delayed_idx);

        WheelAccelMeas(k) = ...
            WheelAccelRawMeas(delayed_idx);

        SuspTravelMeas(k) = ...
            SuspTravelRawMeas(delayed_idx);

        RelVelocityMeas(k) = ...
            RelVelocityRawMeas(delayed_idx);

        MeasurementUpdated(k) = true;

    else

        % Zero-order hold between sensor updates.
        BodyAccelMeas(k) = BodyAccelMeas(k-1);
        WheelAccelMeas(k) = WheelAccelMeas(k-1);
        SuspTravelMeas(k) = SuspTravelMeas(k-1);
        RelVelocityMeas(k) = RelVelocityMeas(k-1);

    end

    % Causal normal RMS features from measurements actually
    % available to the scheduler.
    i0 = max(1,k-window_samples+1);

    BodyRMS(k) = ...
        sqrt(mean(BodyAccelMeas(i0:k).^2));

    WheelRMS(k) = ...
        sqrt(mean(WheelAccelMeas(i0:k).^2));

    TravelRMS(k) = ...
        sqrt(mean(SuspTravelMeas(i0:k).^2));

    % Fast RMS is only an EVENT-ENTRY feature.
    i0_fast = max(1,k-fast_window_samples+1);

    WheelRMSFast(k) = ...
        sqrt(mean(WheelAccelMeas(i0_fast:k).^2));

    % Raw 0.88 m/s shock detector remains available for EVENT ENTRY.
    shock_detected = ...
        abs(RelVelocityMeas(k)) > Shock_RelVel_Threshold;

    % Dedicated lower threshold is used only for SHOCK CONFIRMATION
    % after a transient has already started.
    shock_confirm_candidate = ...
        abs(RelVelocityMeas(k)) > Shock_Confirm_RelVel_Threshold;

    fast_event_detected = ...
        WheelRMSFast(k) > Wheel_Fast_Protection_Threshold;

    ShockDetected(k) = shock_detected;
    ShockConfirmCandidate(k) = shock_confirm_candidate;
    FastEventDetected(k) = fast_event_detected;

    % -----------------------------------------------------
    % Noise-robust shock confirmation.
    %
    % Only evaluated during the fixed 60 ms Protection phase.
    % A shock is confirmed after THREE consecutive measured
    % relative-velocity samples exceed 0.82 m/s.
    % -----------------------------------------------------

    if transient_sequence_active && ...
            protection_counter > 0 && ...
            ~shock_seen_in_transient

        % "3 consecutive samples" means 3 consecutive NEW SENSOR
        % measurements. Held values between updates do not count.
        if MeasurementUpdated(k)

            if shock_confirm_candidate

                shock_confirm_counter = ...
                    shock_confirm_counter + 1;

            else

                shock_confirm_counter = 0;

            end

        end

        if shock_confirm_counter >= Shock_Confirm_Samples

            shock_seen_in_transient = true;
            shock_counter = shock_hold_samples;
            ShockConfirmed(k) = true;

        end

    end

    % Finish the current transient sequence when its requested phases end.
    if transient_sequence_active && ...
            protection_counter == 0 && ...
            (~shock_seen_in_transient || shock_counter == 0)

        transient_sequence_active = false;
        shock_seen_in_transient = false;
        shock_confirm_counter = 0;

    end

    % Re-arm only after the old event has physically cleared.
    if ~transient_sequence_active && ...
            ~fast_event_detected && ...
            ~shock_detected

        transient_armed = true;

    end

    % Fast RMS / shock can START a transient, but cannot keep restarting it.
    if transient_armed && ...
            ~transient_sequence_active && ...
            (fast_event_detected || shock_detected)

        transient_sequence_active = true;
        transient_armed = false;
        transient_start_count = transient_start_count + 1;
        TransientStarted(k) = true;

        protection_counter = protection_hold_samples;

        % A new transient always begins with Protection.
        % Shock follow-up is granted only after the dedicated
        % 0.82 m/s / 3-sample confirmation rule succeeds.
        shock_seen_in_transient = false;
        shock_counter = 0;
        shock_confirm_counter = 0;

    end

    % -----------------------------------------------------
    % Controller-selection hierarchy
    % -----------------------------------------------------

    if transient_sequence_active && protection_counter > 0

        next_mode = 4;      % 60 ms transient Protection

    elseif transient_sequence_active && ...
            shock_seen_in_transient && shock_counter > 0

        next_mode = 2;      % Comfort follow-up after confirmed shock

    elseif WheelRMS(k) > Wheel_High_Threshold

        % Sustained high-frequency activity.
        % IMPORTANT: this uses the normal RMS, NOT the fast entry detector.
        next_mode = 4;

    elseif k < window_samples

        next_mode = 1;      % Balanced startup

    elseif BodyRMS(k) < Body_Low_Threshold && ...
            TravelRMS(k) < Travel_Low_Threshold && ...
            WheelRMS(k) < Wheel_LowForce_Max

        % High-confidence calm/economy region.
        next_mode = 3;      % Low Force

    elseif WheelRMS(k) < Wheel_Comfort_Max && ...
            (BodyRMS(k) >= Body_Low_Threshold || ...
             TravelRMS(k) >= Travel_Comfort_Min)

        % Body-dominated response:
        % the body/suspension needs active damping, but wheel
        % activity is still low enough that Protection is unnecessary.
        next_mode = 2;      % Comfort

    else

        % Mixed / ambiguous dynamics:
        % not calm enough for Low Force,
        % not body-dominated enough for Comfort,
        % and not severe enough for Protection.
        next_mode = 1;      % Balanced

    end

    mode(k) = next_mode;
    K_next = K_bank(next_mode,:);

    % Propagate vehicle to next sensor sample
    if k < N

        tspan = [t(k),t(k+1)];

        if isinf(Fdotmax)

            if isinf(Fmax)

                % Exact ideal V1 closed-loop dynamics.
                odefun = @(tt,xx) ...
                    (A-B*K_next)*xx ...
                    + E*roadFcn(v*tt);

            else

                % Force-magnitude-limited V1.
                odefun = @(tt,xx) ...
                    A*xx ...
                    + B*max(-Fmax,min(Fmax,-K_next*xx)) ...
                    + E*roadFcn(v*tt);

            end

            [~,x_temp] = ode45(odefun,tspan,xk);
            x(k+1,:) = x_temp(end,:);

        else

            % -------------------------------------------------
            % Digital force-rate-limited actuator
            %
            % The newly selected controller requests a force
            % target at the current sample. The target is first
            % clipped to +/-Fmax, then the actuator is allowed
            % to move toward it by at most Fdotmax*Ts.
            %
            % During the next 2 ms interval the physical force
            % ramps linearly from Fa_k to Fa_next. Therefore the
            % actual continuous force slope is bounded by Fdotmax.
            % -------------------------------------------------

            Fa_target = ...
                max(-Fmax,min(Fmax,-K_next*xk));

            max_delta_F = Fdotmax*Ts;

            requested_delta_F = ...
                Fa_target - Fa_k;

            applied_delta_F = ...
                max(-max_delta_F, ...
                min(max_delta_F,requested_delta_F));

            Fa_next = ...
                Fa_k + applied_delta_F;

            RateLimited(k) = ...
                abs(requested_delta_F) > max_delta_F;

            force_slope = ...
                (Fa_next-Fa_k)/Ts;

            t0_interval = t(k);

            odefun = @(tt,xx) ...
                A*xx ...
                + B*(Fa_k + force_slope*(tt-t0_interval)) ...
                + E*roadFcn(v*tt);

            [~,x_temp] = ode45(odefun,tspan,xk);
            x(k+1,:) = x_temp(end,:);

            Fa_state = Fa_next;

        end

    end

    current_mode = next_mode;
    K_current = K_next;

    % Protection timer counts first; Comfort timer counts afterwards.
    if protection_counter > 0

        protection_counter = protection_counter - 1;

    elseif shock_counter > 0

        shock_counter = shock_counter - 1;

    end

end

road_position = v*t;
road = roadFcn(road_position);
TireDef = x(:,3) - road;

sim.t = t;
sim.road_position = road_position;
sim.road = road;
sim.x = x;
sim.FaCmd = FaCmd;
sim.Fa = Fa;

sim.Saturated = Saturated;
sim.RateLimited = RateLimited;

sim.Fmax = Fmax;
sim.Fdotmax = Fdotmax;

sim.BodyAccel = BodyAccel;
sim.WheelAccel = WheelAccel;
sim.SuspTravel = SuspTravel;
sim.TireDef = TireDef;
sim.RelVelocity = RelVelocity;

sim.BodyAccelRawMeas = BodyAccelRawMeas;
sim.WheelAccelRawMeas = WheelAccelRawMeas;
sim.SuspTravelRawMeas = SuspTravelRawMeas;
sim.RelVelocityRawMeas = RelVelocityRawMeas;

sim.BodyAccelMeas = BodyAccelMeas;
sim.WheelAccelMeas = WheelAccelMeas;
sim.SuspTravelMeas = SuspTravelMeas;
sim.RelVelocityMeas = RelVelocityMeas;

sim.MeasurementUpdated = MeasurementUpdated;
sim.sensorDelaySamples = sensorDelaySamples;
sim.sensorUpdateSamples = sensorUpdateSamples;

sim.BodyRMS = BodyRMS;
sim.WheelRMS = WheelRMS;
sim.WheelRMSFast = WheelRMSFast;
sim.TravelRMS = TravelRMS;
sim.mode = mode;

sim.ShockDetected = ShockDetected;
sim.ShockConfirmCandidate = ShockConfirmCandidate;
sim.ShockConfirmed = ShockConfirmed;
sim.FastEventDetected = FastEventDetected;
sim.TransientStarted = TransientStarted;

sim.transient_start_count = transient_start_count;

end


function sim = simulateManualReference( ...
    v, ...
    road_length, ...
    roadFcn, ...
    Ts, ...
    x0, ...
    A, B, E, ...
    K_bank, ...
    ms, mu, ks, cs, kt)

% Model 3A uses known road position and manually assigned controllers.

t_end = road_length/v;
t = (0:Ts:t_end)';
N = length(t);

x = zeros(N,4);
x(1,:) = x0';

Fa = zeros(N,1);
BodyAccel = zeros(N,1);
SuspTravel = zeros(N,1);
mode = zeros(N,1);

for k = 1:N

    xk = x(k,:)';
    road_position_k = v*t(k);
    zr = roadFcn(road_position_k);

    mode_k = manualMode(road_position_k);
    K = K_bank(mode_k,:);

    zs = xk(1);
    zs_dot = xk(2);
    zu = xk(3);
    zu_dot = xk(4);

    Fa_k = -K*xk;

    body_accel = ...
        (-ks*zs ...
        -cs*zs_dot ...
        +ks*zu ...
        +cs*zu_dot ...
        +Fa_k)/ms;

    Fa(k) = Fa_k;
    BodyAccel(k) = body_accel;
    SuspTravel(k) = zs - zu;
    mode(k) = mode_k;

    if k < N

        tspan = [t(k),t(k+1)];

        odefun = @(tt,xx) ...
            (A-B*K)*xx ...
            + E*roadFcn(v*tt);

        [~,x_temp] = ode45(odefun,tspan,xk);
        x(k+1,:) = x_temp(end,:);

    end

end

road_position = v*t;
road = roadFcn(road_position);
TireDef = x(:,3) - road;

sim.t = t;
sim.road_position = road_position;
sim.road = road;
sim.x = x;
sim.Fa = Fa;
sim.BodyAccel = BodyAccel;
sim.SuspTravel = SuspTravel;
sim.TireDef = TireDef;
sim.mode = mode;

end


function mode = manualMode(road_position)

if road_position < 16
    mode = 3;      % Low Force
elseif road_position < 32
    mode = 2;      % Comfort
elseif road_position < 44
    mode = 3;      % Low Force
elseif road_position < 54
    mode = 4;      % Protection
elseif road_position < 64
    mode = 2;      % Comfort
elseif road_position < 72
    mode = 4;      % Protection
else
    mode = 3;      % Low Force
end

end


function zr = compositeRoad(x, lambda_wh)

zr = zeros(size(x));

idx = (x >= 0) & (x < 16);
zr(idx) = 0.002*sin(2*pi*x(idx)/8);

idx = (x >= 16) & (x < 32);
zr(idx) = 0.015*sin(2*pi*(x(idx)-16)/8);

idx = (x >= 32) & (x < 44);
zr(idx) = 0.010*sin(2*pi*(x(idx)-32)/2);

idx = (x >= 44) & (x < 54);
zr(idx) = 0.005*sin(2*pi*(x(idx)-44)/1);

idx = (x >= 58) & (x <= 59.2);
zr(idx) = (0.03/2) .* ...
    (1-cos(2*pi*(x(idx)-58)/1.2));

idx = (x >= 64) & (x < 72);
zr(idx) = 0.005*sin(2*pi*(x(idx)-64)/lambda_wh);

idx = (x >= 72) & (x <= 80);
zr(idx) = 0.002*sin(2*pi*(x(idx)-72)/8);

end


function zr = isolatedBumpRoad( ...
    x, ...
    bump_height, ...
    bump_length, ...
    bump_start)

zr = zeros(size(x));

idx = ...
    x >= bump_start & ...
    x <= bump_start+bump_length;

zr(idx) = ...
    (bump_height/2) .* ...
    (1-cos(2*pi*(x(idx)-bump_start)/bump_length));

end

function sim = simulateFixedController( ...
    v, ...
    road_length, ...
    roadFcn, ...
    Ts, ...
    x0, ...
    A, B, E, ...
    K, ...
    ms, mu, ks, cs, kt, ...
    varargin)

% Fixed-gain baseline simulator used only for robustness comparison.


% Optional actuator limits so the comparison uses the same hardware.
if isempty(varargin)

    Fmax_fixed = Inf;
    Fdotmax_fixed = Inf;

elseif length(varargin) == 1

    Fmax_fixed = varargin{1};
    Fdotmax_fixed = Inf;

else

    Fmax_fixed = varargin{1};
    Fdotmax_fixed = varargin{2};

end


t_end = road_length/v;
t = (0:Ts:t_end)';
N = length(t);

x = zeros(N,4);
x(1,:) = x0';

Fa = zeros(N,1);
BodyAccel = zeros(N,1);
SuspTravel = zeros(N,1);

Fa_state_fixed = 0;

for k = 1:N

    xk = x(k,:)';

    zs = xk(1);
    zs_dot = xk(2);
    zu = xk(3);
    zu_dot = xk(4);

    zr = roadFcn(v*t(k));

    Fa_cmd_k = -K*xk;

    if isinf(Fdotmax_fixed)

        Fa_k = ...
            max(-Fmax_fixed,min(Fmax_fixed,Fa_cmd_k));

    else

        Fa_k = Fa_state_fixed;

    end

    body_accel = ...
        (-ks*zs ...
        -cs*zs_dot ...
        +ks*zu ...
        +cs*zu_dot ...
        +Fa_k)/ms;

    Fa(k) = Fa_k;
    BodyAccel(k) = body_accel;
    SuspTravel(k) = zs - zu;

    if k < N

        tspan = [t(k),t(k+1)];

        if isinf(Fdotmax_fixed)

            if isinf(Fmax_fixed)

                odefun = @(tt,xx) ...
                    (A-B*K)*xx ...
                    + E*roadFcn(v*tt);

            else

                odefun = @(tt,xx) ...
                    A*xx ...
                    + B*max(-Fmax_fixed,min(Fmax_fixed,-K*xx)) ...
                    + E*roadFcn(v*tt);

            end

            [~,x_temp] = ode45(odefun,tspan,xk);
            x(k+1,:) = x_temp(end,:);

        else

            Fa_target_fixed = ...
                max(-Fmax_fixed,min(Fmax_fixed,-K*xk));

            max_delta_fixed = ...
                Fdotmax_fixed*Ts;

            delta_fixed = ...
                max(-max_delta_fixed, ...
                min(max_delta_fixed, ...
                Fa_target_fixed-Fa_k));

            Fa_next_fixed = ...
                Fa_k + delta_fixed;

            force_slope_fixed = ...
                (Fa_next_fixed-Fa_k)/Ts;

            t0_fixed = t(k);

            odefun = @(tt,xx) ...
                A*xx ...
                + B*(Fa_k + ...
                force_slope_fixed*(tt-t0_fixed)) ...
                + E*roadFcn(v*tt);

            [~,x_temp] = ode45(odefun,tspan,xk);
            x(k+1,:) = x_temp(end,:);

            Fa_state_fixed = Fa_next_fixed;

        end

    end

end

road_position = v*t;
road = roadFcn(road_position);
TireDef = x(:,3) - road;

sim.t = t;
sim.road_position = road_position;
sim.road = road;
sim.x = x;
sim.Fa = Fa;
sim.BodyAccel = BodyAccel;
sim.SuspTravel = SuspTravel;
sim.TireDef = TireDef;

end


function zr = mixedRoadProfile( ...
    x, ...
    profile_id, ...
    road_start, ...
    road_end, ...
    ramp_length)

% Deterministic unseen multi-sine roads.
% Wavelengths and phase combinations differ from the development road.

switch profile_id

    case 1      % Mixed Mild

        amp = [0.0015 0.0010 0.0005];
        lambda = [7.0 3.1 1.4];
        phase = [0.15 0.90 1.70];

    case 2      % Mixed Medium

        amp = [0.0030 0.0020 0.0010];
        lambda = [6.2 2.4 1.1];
        phase = [0.35 1.20 2.10];

    case 3      % Mixed Rough

        amp = [0.0050 0.0030 0.0015];
        lambda = [5.4 1.8 0.9];
        phase = [0.55 1.45 2.40];

    otherwise

        error('Unknown mixed-road profile ID.')

end


zr_raw = ...
    amp(1)*sin(2*pi*x/lambda(1) + phase(1)) ...
    + amp(2)*sin(2*pi*x/lambda(2) + phase(2)) ...
    + amp(3)*sin(2*pi*x/lambda(3) + phase(3));


% Smooth spatial envelope:
% 0 before road_start,
% cosine ramp-up,
% full amplitude,
% cosine ramp-down,
% 0 after road_end.

envelope = zeros(size(x));

idx_up = ...
    x >= road_start & ...
    x < road_start+ramp_length;

envelope(idx_up) = ...
    0.5*(1-cos( ...
    pi*(x(idx_up)-road_start)/ramp_length));


idx_full = ...
    x >= road_start+ramp_length & ...
    x <= road_end-ramp_length;

envelope(idx_full) = 1;


idx_down = ...
    x > road_end-ramp_length & ...
    x <= road_end;

envelope(idx_down) = ...
    0.5*(1+cos( ...
    pi*(x(idx_down)-(road_end-ramp_length))/ramp_length));


zr = envelope .* zr_raw;

end
