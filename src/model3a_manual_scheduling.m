%% MODEL 3A - IDEAL GAIN-SCHEDULED ACTIVE SUSPENSION

clear;
clc;
close all;


%% 1. Quarter-Car Plant

ms = 300;        % Sprung mass [kg]
mu = 40;         % Unsprung mass [kg]
ks = 22000;      % Suspension stiffness [N/m]
cs = 1500;       % Suspension damping [N*s/m]
kt = 200000;     % Tire stiffness [N/m]


%% 2. State-Space Model

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


%% 3. LQR PERFORMANCE OUTPUTS

Ca = [-ks/ms, -cs/ms, ks/ms, cs/ms];
Da = 1/ms;

Cs = [1, 0, -1, 0];


%% 4. LQR CONTROLLER BANK

controller_names = [ ...
    "Balanced"
    "Comfort"
    "Low Force"
    "Protection"
    ];

% Reference values for each controller
a_ref_bank = [ ...
    2.0
    1.2
    2.0
    3.0
    ];

s_ref_bank = [ ...
    0.020
    0.030
    0.020
    0.012
    ];

F_ref_bank = [ ...
    500
    800
    350
    300
    ];

nK = length(controller_names);

% Storage for controller gains
K_bank = zeros(nK,4);


%% Generate LQR gains

for j = 1:nK

    a_ref_j = a_ref_bank(j);
    s_ref_j = s_ref_bank(j);
    F_ref_j = F_ref_bank(j);

    % Cost weights
    wa_j = 1/a_ref_j^2;
    ws_j = 1/s_ref_j^2;
    wF_j = 1/F_ref_j^2;

    % Generalized LQR matrices
    Q_j = wa_j*(Ca'*Ca) ...
        + ws_j*(Cs'*Cs);

    N_j = wa_j*(Ca'*Da);

    R_j = wa_j*(Da^2) ...
        + wF_j;

    % Controller gain
    K_bank(j,:) = ...
        lqr(A,B,Q_j,R_j,N_j);

end


K_balanced   = K_bank(1,:);
K_comfort    = K_bank(2,:);
K_lowforce   = K_bank(3,:);
K_protection = K_bank(4,:);


%% 5. MODEL 3A - COMPOSITE VALIDATION ROAD

v = 10;                 % Constant vehicle speed [m/s]
road_length = 80;       % Total road length [m]

% Wheel-hop wavelength corresponding to 11.25 Hz at v = 10 m/s
lambda_wh = v/11.25;


% Composite road function handle
zr_composite = @(x) compositeRoad(x, lambda_wh);
% ---------------------------------------------------------
% Composite road
%
% 0-16 m   : Mild smooth road
% 16-32 m  : Long-wave undulation
% 32-44 m  : Medium-frequency road
% 44-54 m  : High-frequency rough road
% 54-64 m  : Single bump section
% 64-72 m  : Wheel-hop region
% 72-80 m  : Mild smooth road
% ---------------------------------------------------------

%% 5.1 Plot Composite Road

x_plot = linspace(0,road_length,6000);
zr_plot = zr_composite(x_plot);

figure

plot(x_plot,1000*zr_plot,'LineWidth',1.4)
grid on
hold on

xline(16,'--')
xline(32,'--')
xline(44,'--')
xline(54,'--')
xline(64,'--')
xline(72,'--')

xlabel('Road Position x (m)')
ylabel('Road Height (mm)')
title('Model 3A - Composite Validation Road')

xlim([0 road_length])

%% 6. MODEL 3A - IDEAL MANUAL GAIN SCHEDULING

t_end = road_length / v;

% Small step required for bump and wheel-hop sections
ode_options = odeset( ...
    'MaxStep', 0.002, ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-8);

% Scheduled closed-loop system
odefun_sched = @(t,x) scheduledODE( ...
    t, x, ...
    A, B, E, ...
    zr_composite, v, ...
    K_lowforce, ...
    K_comfort, ...
    K_protection);

% Simulate
[t_sched, x_sched] = ode45( ...
    odefun_sched, ...
    [0 t_end], ...
    x0, ...
    ode_options);


%% 6.1 Reconstruct Scheduled Controller Response

n_sched = length(t_sched);

Fa_sched = zeros(n_sched,1);
mode_sched = zeros(n_sched,1);

for i = 1:n_sched

    road_position = v*t_sched(i);

    [K_now, mode_now] = selectGain( ...
        road_position, ...
        K_lowforce, ...
        K_comfort, ...
        K_protection);

    Fa_sched(i) = ...
        -K_now*x_sched(i,:)';

    mode_sched(i) = mode_now;

end

% Road input experienced by vehicle
zr_sched = zr_composite(v*t_sched);

% Body acceleration
Accel_sched = ...
    (Ca*x_sched')' + Da*Fa_sched;

% Suspension travel
SuspTravel_sched = ...
    x_sched(:,1) - x_sched(:,3);

% Tire deflection
TireDef_sched = ...
    x_sched(:,3) - zr_sched;


%% 6.2 Check Scheduled Controller Switching

figure

subplot(3,1,1)

plot(v*t_sched, ...
    1000*zr_sched, ...
    'LineWidth',1.3)

grid on
ylabel('Road Height (mm)')
title('Model 3A - Ideal Gain Scheduling')


subplot(3,1,2)

plot(v*t_sched, ...
    Accel_sched, ...
    'LineWidth',1.2)

grid on
ylabel('Body Accel. (m/s^2)')


subplot(3,1,3)

stairs(v*t_sched, ...
    mode_sched, ...
    'LineWidth',1.5)

grid on

yticks([1 2 3])

yticklabels({ ...
    'Low Force', ...
    'Comfort', ...
    'Protection'})

xlabel('Road Position x (m)')
ylabel('Active Controller')

ylim([0.5 3.5])





%% 7. FIXED CONTROLLER BENCHMARKS

% Storage for fixed-controller responses
t_fixed = cell(nK,1);
Accel_fixed = cell(nK,1);
Force_fixed = cell(nK,1);

% Performance metrics
Accel_RMS_all        = zeros(nK+1,1);
Accel_Peak_all       = zeros(nK+1,1);
SuspTravel_Peak_all  = zeros(nK+1,1);
TireDef_Peak_all     = zeros(nK+1,1);
Force_RMS_all        = zeros(nK+1,1);
Force_Peak_all       = zeros(nK+1,1);


%% 7.1 ADD SCHEDULED CONTROLLER RESULTS

idx_sched = nK + 1;

Accel_RMS_all(idx_sched) = ...
    rms(Accel_sched);

Accel_Peak_all(idx_sched) = ...
    max(abs(Accel_sched));

SuspTravel_Peak_all(idx_sched) = ...
    max(abs(SuspTravel_sched));

TireDef_Peak_all(idx_sched) = ...
    max(abs(TireDef_sched));

Force_RMS_all(idx_sched) = ...
    rms(Fa_sched);

Force_Peak_all(idx_sched) = ...
    max(abs(Fa_sched));

%% Run all four fixed controllers

for j = 1:nK

    K_current = K_bank(j,:);

    % Fixed closed-loop system
    odefun_fixed = @(t,x) ...
        (A-B*K_current)*x ...
        + E*zr_composite(v*t);

    % Simulation
    [t_k,x_k] = ode45( ...
        odefun_fixed, ...
        [0 t_end], ...
        x0, ...
        ode_options);

    % Road input
    zr_k = zr_composite(v*t_k);

    % Actuator force
    Fa_k = -(K_current*x_k')';

    % Body acceleration
    Accel_k = ...
        (Ca*x_k')' + Da*Fa_k;

    % Suspension travel
    SuspTravel_k = ...
        x_k(:,1) - x_k(:,3);

    % Tire deflection
    TireDef_k = ...
        x_k(:,3) - zr_k;

    % Save responses for plotting
    t_fixed{j} = t_k;
    Accel_fixed{j} = Accel_k;
    Force_fixed{j} = Fa_k;

    % Global performance metrics
    Accel_RMS_all(j) = rms(Accel_k);

    Accel_Peak_all(j) = ...
        max(abs(Accel_k));

    SuspTravel_Peak_all(j) = ...
        max(abs(SuspTravel_k));

    TireDef_Peak_all(j) = ...
        max(abs(TireDef_k));

    Force_RMS_all(j) = rms(Fa_k);

    Force_Peak_all(j) = ...
        max(abs(Fa_k));

end


%% 7.2 GLOBAL PERFORMANCE COMPARISON

SystemNames = [ ...
    controller_names
    "Scheduled"
    ];

ComparisonResults = table( ...
    SystemNames, ...
    Accel_RMS_all, ...
    Accel_Peak_all, ...
    SuspTravel_Peak_all, ...
    TireDef_Peak_all, ...
    Force_RMS_all, ...
    Force_Peak_all, ...
    'VariableNames', { ...
    'Controller', ...
    'Accel_RMS', ...
    'Accel_Peak', ...
    'SuspTravel_Peak', ...
    'TireDef_Peak', ...
    'Force_RMS', ...
    'Force_Peak'});

disp(ComparisonResults)


%% 7.3 BODY ACCELERATION COMPARISON

figure
hold on

% Four fixed controllers
for j = 1:nK

    plot( ...
        v*t_fixed{j}, ...
        Accel_fixed{j}, ...
        'LineWidth',1.0);

end

% Scheduled controller
plot( ...
    v*t_sched, ...
    Accel_sched, ...
    'LineWidth',2.0);

% Road segment boundaries
xline(16,'--','HandleVisibility','off')
xline(32,'--','HandleVisibility','off')
xline(44,'--','HandleVisibility','off')
xline(54,'--','HandleVisibility','off')
xline(64,'--','HandleVisibility','off')
xline(72,'--','HandleVisibility','off')

grid on

xlabel('Road Position x (m)')
ylabel('Body Acceleration (m/s^2)')

title('Model 3A - Fixed vs Scheduled LQR')

legend( ...
    SystemNames, ...
    'Location','best')

xlim([0 road_length])



%% 7.4 ROAD PROFILE AND SELECTED CONTROL OBJECTIVE

road_position_sched = v*t_sched;

% Boundaries between manually defined operating regions
segment_edges = [16 32 44 54 64 72];

% Approximate center of each segment for labels
segment_centers = [8 24 38 49 59 68 76];

segment_names = { ...
    'Mild Smooth', ...
    'Long-Wave', ...
    'Medium Frequency', ...
    'High Frequency', ...
    'Bump Section', ...
    'Wheel-Hop Test', ...
    'Mild Smooth'};


figure


%% Road profile
subplot(2,1,1)

plot( ...
    road_position_sched, ...
    1000*zr_sched, ...
    'LineWidth',1.3)

grid on
hold on

% Show segment boundaries
for xb = segment_edges
    xline(xb,'--','HandleVisibility','off')
end

ylabel('Road Height (mm)')
title('Model 3A - Road Profile and Selected Control Objective')

xlim([0 road_length])


% Add road-condition labels
yl = ylim;

label_y = yl(2) - 0.08*(yl(2)-yl(1));

for i = 1:length(segment_centers)

    text( ...
        segment_centers(i), ...
        label_y, ...
        segment_names{i}, ...
        'HorizontalAlignment','center', ...
        'VerticalAlignment','top', ...
        'FontSize',8);

end



%% Selected controller objective
subplot(2,1,2)

stairs( ...
    road_position_sched, ...
    mode_sched, ...
    'LineWidth',1.8)

grid on
hold on

% Same segment boundaries
for xb = segment_edges
    xline(xb,'--','HandleVisibility','off')
end

yticks([1 2 3])

yticklabels({ ...
    'Low Force - Actuator Economy', ...
    'Comfort - Ride Comfort', ...
    'Protection - System Protection'})

xlabel('Road Position x (m)')
ylabel('Selected Objective')

ylim([0.5 3.5])
xlim([0 road_length])



%% 7.5 SCHEDULED PERFORMANCE RELATIVE TO FIXED BASELINES

idx_balanced = find(SystemNames == "Balanced");
idx_comfort  = find(SystemNames == "Comfort");
idx_sched    = find(SystemNames == "Scheduled");

metrics_sched = [ ...
    Accel_RMS_all(idx_sched);
    Accel_Peak_all(idx_sched);
    SuspTravel_Peak_all(idx_sched);
    TireDef_Peak_all(idx_sched);
    Force_RMS_all(idx_sched);
    Force_Peak_all(idx_sched)];

metrics_balanced = [ ...
    Accel_RMS_all(idx_balanced);
    Accel_Peak_all(idx_balanced);
    SuspTravel_Peak_all(idx_balanced);
    TireDef_Peak_all(idx_balanced);
    Force_RMS_all(idx_balanced);
    Force_Peak_all(idx_balanced)];

metrics_comfort = [ ...
    Accel_RMS_all(idx_comfort);
    Accel_Peak_all(idx_comfort);
    SuspTravel_Peak_all(idx_comfort);
    TireDef_Peak_all(idx_comfort);
    Force_RMS_all(idx_comfort);
    Force_Peak_all(idx_comfort)];

Change_vs_Balanced = ...
    100*(metrics_sched./metrics_balanced - 1);

Change_vs_Comfort = ...
    100*(metrics_sched./metrics_comfort - 1);

Metric = [ ...
    "Acceleration RMS";
    "Acceleration Peak";
    "Suspension Travel Peak";
    "Tire Deflection Peak";
    "Actuator Force RMS";
    "Actuator Force Peak"];

ScheduledComparison = table( ...
    Metric, ...
    Change_vs_Balanced, ...
    Change_vs_Comfort, ...
    'VariableNames', { ...
    'Metric', ...
    'Change_vs_Balanced_percent', ...
    'Change_vs_Comfort_percent'});

disp(ScheduledComparison)




%% LOCAL FUNCTIONS

function zr = compositeRoad(x, lambda_wh)

% Initialize road height
zr = zeros(size(x));

% Segment 1 - Mild Smooth Road: 0-16 m
idx = (x >= 0) & (x < 16);
zr(idx) = 0.002*sin(2*pi*x(idx)/8);

% Segment 2 - Long-Wave Undulation: 16-32 m
idx = (x >= 16) & (x < 32);
zr(idx) = 0.015*sin(2*pi*(x(idx)-16)/8);

% Segment 3 - Medium-Frequency Road: 32-44 m
idx = (x >= 32) & (x < 44);
zr(idx) = 0.010*sin(2*pi*(x(idx)-32)/2);

% Segment 4 - High-Frequency Rough Road: 44-54 m
idx = (x >= 44) & (x < 54);
zr(idx) = 0.005*sin(2*pi*(x(idx)-44)/1);

% Segment 5 - Single Bump: 58-59.2 m
idx = (x >= 58) & (x <= 59.2);
zr(idx) = (0.03/2) * ...
    (1-cos(2*pi*(x(idx)-58)/1.2));

% Segment 6 - Wheel-Hop Region: 64-72 m
idx = (x >= 64) & (x < 72);
zr(idx) = 0.005*sin(2*pi*(x(idx)-64)/lambda_wh);

% Segment 7 - Mild Smooth Road: 72-80 m
idx = (x >= 72) & (x <= 80);
zr(idx) = 0.002*sin(2*pi*(x(idx)-72)/8);

end


function dx = scheduledODE( ...
    t, x, ...
    A, B, E, ...
    zr_composite, v, ...
    K_lowforce, ...
    K_comfort, ...
    K_protection)

% Current vehicle position
road_position = v*t;

% Select controller manually based on known road segment
K = selectGain( ...
    road_position, ...
    K_lowforce, ...
    K_comfort, ...
    K_protection);

% Road disturbance
zr = zr_composite(road_position);

% Scheduled closed-loop dynamics
dx = ...
    (A-B*K)*x ...
    + E*zr;

end




function [K, mode] = selectGain( ...
    road_position, ...
    K_lowforce, ...
    K_comfort, ...
    K_protection)

if road_position < 16

    % Mild smooth road
    K = K_lowforce;
    mode = 1;

elseif road_position < 32

    % Long-wave undulation
    K = K_comfort;
    mode = 2;

elseif road_position < 44

    % Medium-frequency road
    K = K_lowforce;
    mode = 1;

elseif road_position < 54

    % High-frequency rough road
    K = K_protection;
    mode = 3;

elseif road_position < 64

    % Single bump section
    K = K_comfort;
    mode = 2;

elseif road_position < 72

    % Wheel-hop region
    K = K_protection;
    mode = 3;

else

    % Final mild smooth road
    K = K_lowforce;
    mode = 1;

end

end
