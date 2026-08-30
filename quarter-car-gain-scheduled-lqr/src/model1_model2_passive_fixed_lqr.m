%% QUARTER-CAR SUSPENSION PROJECT
% Model 1: Passive suspension
% Model 2: Fixed-LQR active suspension + s_ref sweep
% Animation: Fixed-LQR response on an irregular road

clear;
clc;
close all;


%% 1. QUARTER-CAR PLANT

% Nominal passenger-car parameters
ms = 300;        % Sprung mass [kg]
mu = 40;         % Unsprung mass [kg]
ks = 22000;      % Suspension stiffness [N/m]
cs = 1500;       % Suspension damping [N*s/m]
kt = 200000;     % Tire stiffness [N/m]

% State vector:
% x = [zs; zs_dot; zu; zu_dot]

A = [0,        1,             0,               0;
    -ks/ms,   -cs/ms,         ks/ms,           cs/ms;
     0,        0,             0,               1;
     ks/mu,    cs/mu,        -(ks+kt)/mu,     -cs/mu];

% Active actuator input Fa
B = [0;
     1/ms;
     0;
    -1/mu];

% Road displacement input zr
E = [0;
     0;
     0;
     kt/mu];

% Initial condition: static-equilibrium coordinates
x0 = [0; 0; 0; 0];


%% 2. MODEL 1 - PASSIVE SUSPENSION

% Baseline sinusoidal road input
Ar = 0.02;       % Road amplitude [m]
f  = 10;         % Excitation frequency [Hz]
tspan = [0 5];

zr_baseline = @(t) Ar*sin(2*pi*f*t);

% Passive system: Fa = 0
odefun_passive = @(t,x) A*x + E*zr_baseline(t);

[t, x] = ode45(odefun_passive, tspan, x0);

% States
zs     = x(:,1);
zs_dot = x(:,2);
zu     = x(:,3);
zu_dot = x(:,4);

% Road displacement
zr = zr_baseline(t);

% Body acceleration
zs_ddot = (-ks*zs ...
           -cs*zs_dot ...
           +ks*zu ...
           +cs*zu_dot)/ms;

% Performance variables
susp_travel     = zs - zu;
tire_deflection = zu - zr;


%% 3. MODEL 2 - FIXED LQR

% Reference performance levels
a_ref = 2;         % Body acceleration reference [m/s^2]
s_ref = 0.020;     % Suspension travel reference [m]
F_ref = 500;       % Actuator force reference [N]

% Cost weights
wa = 1/a_ref^2;
ws = 1/s_ref^2;
wF = 1/F_ref^2;

% Body acceleration:
% zs_ddot = Ca*x + Da*Fa
Ca = [-ks/ms, -cs/ms, ks/ms, cs/ms];
Da = 1/ms;

% Suspension travel:
% zs - zu = Cs*x
Cs = [1, 0, -1, 0];

% Convert the physical cost into generalized LQR form
Q = wa*(Ca'*Ca) + ws*(Cs'*Cs);
N = wa*(Ca'*Da);
R = wa*(Da^2) + wF;

% Fixed LQR gain
K = lqr(A, B, Q, R, N);

% Active closed-loop simulation
odefun_active = @(t,x) (A-B*K)*x + E*zr_baseline(t);

[t_active, x_active] = ode45(odefun_active, tspan, x0);

% States
zs_active     = x_active(:,1);
zs_dot_active = x_active(:,2);
zu_active     = x_active(:,3);
zu_dot_active = x_active(:,4);

% Road displacement
zr_active = zr_baseline(t_active);

% Actuator force: Fa = -Kx
Fa_active = -(K*x_active')';

% Body acceleration
zs_ddot_active = (-ks*zs_active ...
                  -cs*zs_dot_active ...
                  +ks*zu_active ...
                  +cs*zu_dot_active ...
                  +Fa_active)/ms;

% Performance variables
susp_travel_active     = zs_active - zu_active;
tire_deflection_active = zu_active - zr_active;




%% 4. ANIMATION ROAD PROFILE

% This profile is used only by the visualization below.
% It is intentionally separate from road7 used in the s_ref sweep.

zr_irregular_raw = @(t) ...
    0.0050*sin(2*pi*0.8*t) ...
    + 0.0030*sin(2*pi*2.2*t + 0.6) ...
    + 0.0018*sin(2*pi*4.8*t + 1.4) ...
    + 0.0008*sin(2*pi*8.0*t + 0.3);

% Smooth entrance avoids an artificial initial-condition jump
T_ramp = 0.5;

ramp_irregular = @(t) ...
    ((t >= 0) & (t < T_ramp)) .* ...
    (0.5 .* (1 - cos(pi*t/T_ramp))) ...
    + (t >= T_ramp);

zr_irregular = @(t) ramp_irregular(t).*zr_irregular_raw(t);


%% 5. ACTIVE LQR ANIMATION - IRREGULAR ROAD

% Simulate active system once for the animation
odefun_anim = @(t,x) (A-B*K)*x + E*zr_irregular(t);
[t_anim_raw, x_anim_raw] = ode45(odefun_anim, [0 10], x0);


% --------------------------------------------------
% SMOOTH 60 FPS TIME VECTOR
% --------------------------------------------------

fps = 60;
playback_speed = 1;
t_anim = (0:playback_speed/fps:10)';

% Interpolate ODE45 solution onto equally spaced time points
zs_anim = interp1(t_anim_raw,x_anim_raw(:,1), ...
                  t_anim,'pchip');

zu_anim = interp1(t_anim_raw,x_anim_raw(:,3), ...
                  t_anim,'pchip');

% Road input at same time points
zr_anim = zr_irregular(t_anim);


% --------------------------------------------------
% VISUALIZATION SETTINGS
% --------------------------------------------------
fig = figure( ...
    'Units','pixels', ...
    'Position',[50 50 1920 1080], ...
    'Color','w', ...
    'Renderer','opengl');

scale = 5;
v_car = 1.0;

body_base = 0.32;

% Road spatial coordinates
xroad = linspace(-2,2,1000);

% Vehicle geometry
wheel_radius = 0.055;

unsprung_width  = 0.24;
unsprung_height = 0.06;

body_width  = 0.50;
body_height = 0.08;

% Suspension geometry
spring_x_center = -0.065;
spring_width = 0.018;
n_coils = 8;

damper_x = 0;
damper_width = 0.035;
piston_width = 0.025;

actuator_x = 0.065;
actuator_width = 0.035;


% --------------------------------------------------
% INITIAL FRAME GEOMETRY
% --------------------------------------------------

i = 1;

x_car = v_car*t_anim(i);

world_x = xroad + x_car;
road_time = world_x/v_car;

zroad = zr_irregular(road_time);

zr_under_wheel = zr_anim(i);

tire_def_anim = ...
    zu_anim(i) - zr_anim(i);

wheel_center_y = ...
    scale*zr_under_wheel ...
    + wheel_radius ...
    + scale*tire_def_anim;

unsprung_bottom = ...
    wheel_center_y ...
    + wheel_radius ...
    + 0.03;

zs_vis = ...
    body_base ...
    + scale*zs_anim(i);

suspension_bottom = ...
    unsprung_bottom + unsprung_height;

suspension_top = zs_vis;

susp_height = ...
    suspension_top - suspension_bottom;


% --------------------------------------------------
% DRAW STATIC OBJECTS ONCE
% --------------------------------------------------

hold on
grid on


% ROAD
hRoad = plot( ...
    xroad, ...
    scale*zroad, ...
    'k', ...
    'LineWidth',3);


% WHEEL
hWheel = rectangle( ...
    'Position',[ ...
        -wheel_radius, ...
        wheel_center_y-wheel_radius, ...
        2*wheel_radius, ...
        2*wheel_radius], ...
    'Curvature',[1 1], ...
    'EdgeColor','k', ...
    'LineWidth',2);


% UNSPRUNG MASS
hUnsprung = rectangle( ...
    'Position',[ ...
        -unsprung_width/2, ...
        unsprung_bottom, ...
        unsprung_width, ...
        unsprung_height], ...
    'EdgeColor','k', ...
    'LineWidth',2);


% WHEEL -> UNSPRUNG MASS CONNECTION
hWheelConnection = plot( ...
    [0 0], ...
    [wheel_center_y + wheel_radius, ...
     unsprung_bottom], ...
    'k', ...
    'LineWidth',2);


% SPRUNG MASS
hBody = rectangle( ...
    'Position',[ ...
        -body_width/2, ...
        zs_vis, ...
        body_width, ...
        body_height], ...
    'EdgeColor','k', ...
    'LineWidth',2);


% ==================================================
% SPRING ks
% ==================================================

y_spring = linspace( ...
    suspension_bottom, ...
    suspension_top, ...
    2*n_coils + 1);

x_spring = zeros(size(y_spring));

for k = 2:length(x_spring)-1

    if mod(k,2) == 0
        x_spring(k) = spring_width;
    else
        x_spring(k) = -spring_width;
    end

end

x_spring(1) = 0;
x_spring(end) = 0;

hSpring = plot( ...
    spring_x_center + x_spring, ...
    y_spring, ...
    'k', ...
    'LineWidth',2);


% ==================================================
% DAMPER cs
% ==================================================

damper_body_bottom = ...
    suspension_bottom + 0.30*susp_height;

damper_body_top = ...
    suspension_bottom + 0.58*susp_height;


% Lower rod
hDamperLower = plot( ...
    [damper_x damper_x], ...
    [suspension_bottom damper_body_bottom], ...
    'k', ...
    'LineWidth',2);


% Damper body
hDamperBody = rectangle( ...
    'Position',[ ...
        damper_x-damper_width/2, ...
        damper_body_bottom, ...
        damper_width, ...
        damper_body_top-damper_body_bottom], ...
    'EdgeColor','k', ...
    'LineWidth',2);


% Upper rod
hDamperUpper = plot( ...
    [damper_x damper_x], ...
    [damper_body_top suspension_top], ...
    'k', ...
    'LineWidth',2);


% Piston head
hDamperPiston = plot( ...
    [damper_x-piston_width ...
     damper_x+piston_width], ...
    [damper_body_top damper_body_top], ...
    'k', ...
    'LineWidth',2);


% ==================================================
% ACTIVE ACTUATOR Fa
% ==================================================

actuator_body_bottom = ...
    suspension_bottom + 0.25*susp_height;

actuator_body_top = ...
    suspension_bottom + 0.60*susp_height;


% Lower rod
hActuatorLower = plot( ...
    [actuator_x actuator_x], ...
    [suspension_bottom actuator_body_bottom], ...
    'k', ...
    'LineWidth',2);


% Actuator housing
hActuatorBody = rectangle( ...
    'Position',[ ...
        actuator_x-actuator_width/2, ...
        actuator_body_bottom, ...
        actuator_width, ...
        actuator_body_top-actuator_body_bottom], ...
    'EdgeColor','k', ...
    'LineWidth',2);


% Upper rod
hActuatorUpper = plot( ...
    [actuator_x actuator_x], ...
    [actuator_body_top suspension_top], ...
    'k', ...
    'LineWidth',2);


% Piston head
hActuatorPiston = plot( ...
    [actuator_x-0.015 ...
     actuator_x+0.015], ...
    [actuator_body_top actuator_body_top], ...
    'k', ...
    'LineWidth',2);


% --------------------------------------------------
% FIGURE SETTINGS
% --------------------------------------------------

hTitle = title( ...
    sprintf('Active LQR Suspension - t = %.2f s', ...
    t_anim(1)));

xlabel('Road Position Relative to Vehicle (m)')
ylabel('Vertical Position')

xlim([-0.7 0.7])
ylim([-0.02 0.55])

axis equal


% --------------------------------------------------
% ANIMATION LOOP
% --------------------------------------------------

animation_clock = tic;

for i = 1:length(t_anim)

    % ----------------------------------------------
    % ROAD
    % ----------------------------------------------

    x_car = v_car*t_anim(i);

    world_x = xroad + x_car;

    road_time = world_x/v_car;

    zroad = zr_irregular(road_time);

    set(hRoad, ...
        'YData',scale*zroad);


    % ----------------------------------------------
    % VEHICLE POSITIONS
    % ----------------------------------------------

    zr_under_wheel = zr_anim(i);

    tire_def_anim = ...
        zu_anim(i) - zr_anim(i);

    wheel_center_y = ...
        scale*zr_under_wheel ...
        + wheel_radius ...
        + scale*tire_def_anim;

    unsprung_bottom = ...
        wheel_center_y ...
        + wheel_radius ...
        + 0.03;

    zs_vis = ...
        body_base ...
        + scale*zs_anim(i);


    % ----------------------------------------------
    % WHEEL
    % ----------------------------------------------

    set(hWheel, ...
        'Position',[ ...
            -wheel_radius, ...
            wheel_center_y-wheel_radius, ...
            2*wheel_radius, ...
            2*wheel_radius]);


    % ----------------------------------------------
    % UNSPRUNG MASS
    % ----------------------------------------------

    set(hUnsprung, ...
        'Position',[ ...
            -unsprung_width/2, ...
            unsprung_bottom, ...
            unsprung_width, ...
            unsprung_height]);


    % Wheel connection
    set(hWheelConnection, ...
        'YData',[ ...
            wheel_center_y + wheel_radius, ...
            unsprung_bottom]);


    % ----------------------------------------------
    % SPRUNG MASS
    % ----------------------------------------------

    set(hBody, ...
        'Position',[ ...
            -body_width/2, ...
            zs_vis, ...
            body_width, ...
            body_height]);


    % ----------------------------------------------
    % SUSPENSION LIMITS
    % ----------------------------------------------

    suspension_bottom = ...
        unsprung_bottom + unsprung_height;

    suspension_top = zs_vis;

    susp_height = ...
        suspension_top - suspension_bottom;


    % ----------------------------------------------
    % SPRING ks
    % ----------------------------------------------

    y_spring = linspace( ...
        suspension_bottom, ...
        suspension_top, ...
        2*n_coils + 1);

    set(hSpring, ...
        'YData',y_spring);


    % ----------------------------------------------
    % DAMPER cs
    % ----------------------------------------------

    damper_body_bottom = ...
        suspension_bottom ...
        + 0.30*susp_height;

    damper_body_top = ...
        suspension_bottom ...
        + 0.58*susp_height;

    set(hDamperLower, ...
        'YData',[ ...
            suspension_bottom, ...
            damper_body_bottom]);

    set(hDamperBody, ...
        'Position',[ ...
            damper_x-damper_width/2, ...
            damper_body_bottom, ...
            damper_width, ...
            damper_body_top-damper_body_bottom]);

    set(hDamperUpper, ...
        'YData',[ ...
            damper_body_top, ...
            suspension_top]);

    set(hDamperPiston, ...
        'YData',[ ...
            damper_body_top, ...
            damper_body_top]);


    % ----------------------------------------------
    % ACTIVE ACTUATOR Fa
    % ----------------------------------------------

    actuator_body_bottom = ...
        suspension_bottom ...
        + 0.25*susp_height;

    actuator_body_top = ...
        suspension_bottom ...
        + 0.60*susp_height;

    set(hActuatorLower, ...
        'YData',[ ...
            suspension_bottom, ...
            actuator_body_bottom]);

    set(hActuatorBody, ...
        'Position',[ ...
            actuator_x-actuator_width/2, ...
            actuator_body_bottom, ...
            actuator_width, ...
            actuator_body_top-actuator_body_bottom]);

    set(hActuatorUpper, ...
        'YData',[ ...
            actuator_body_top, ...
            suspension_top]);

    set(hActuatorPiston, ...
        'YData',[ ...
            actuator_body_top, ...
            actuator_body_top]);


    % ----------------------------------------------
    % TITLE
    % ----------------------------------------------

    set(hTitle, ...
        'String',sprintf( ...
        'Active LQR Suspension - t = %.2f s', ...
        t_anim(i)));


    % ----------------------------------------------
    % DRAW FRAME
    % ----------------------------------------------

    drawnow



    % Keep playback approximately real-time at 60 FPS
    next_frame_time = i/(fps);

    remaining_time = ...
        next_frame_time - toc(animation_clock);

    if remaining_time > 0
        pause(remaining_time)
    end


end
