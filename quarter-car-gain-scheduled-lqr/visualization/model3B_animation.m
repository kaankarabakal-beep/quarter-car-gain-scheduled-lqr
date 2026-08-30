%% MODEL 3B - COMPOSITE + MULTI-MODE MULTI-SINE ANIMATION / VIDEO TEASER - V11
% Standalone visualization of the FINAL locked Model 3B controller.
% It does NOT run the full model3B.m report/robustness script.
% Instead, it rebuilds the same plant, LQR bank, locked scheduler,
% actuator/sensor settings, simulates one 10-second showcase road,
% and immediately creates the animation/video.
%
% The road preserves the standard composite-road sequence up to the bump,
% then replaces the visually similar wheel-hop segment with a mild smooth
% recovery section followed by an irregular multi-sine ending.
% Default speed: 10 m/s, road length: 100 m -> 10 s simulation.

clear;
clc;
close all;

%% ---------------------------------------------------------------
% SHOWCASE COMPOSITE + MULTI-SINE ROAD
% ---------------------------------------------------------------
%
% 0--16 m    : original mild sine
% 16--32 m   : original long-wave section
% 32--44 m   : original medium-frequency section
% 44--54 m   : original high-frequency section
% 54--58 m   : flat approach to bump
% 58--59.2 m : original 30 mm raised-cosine bump
% 59.2--68 m : mild smooth recovery road
% 68--70 m   : smooth blend into body-dominated multi-sine
% 70--78 m   : body-dominated multi-sine (designed to invite Comfort)
% 78--80 m   : blend into mixed multi-sine
% 80--88 m   : mixed multi-sine (Balanced region)
% 88--90 m   : blend into wheel-active multi-sine
% 90--98 m   : wheel-active irregular multi-sine (designed to invite Protection)
% 98--100 m  : smooth fade to zero
%
% The original wheel-hop section is intentionally omitted because its
% appearance is too similar to the high-frequency section in the animation.
% No controller, scheduler, actuator, or sensor parameter is changed.

wanted_speed = 10;              % [m/s]
showcase_name = "Composite + Multi-Mode Multi-Sine Showcase";

%% ---------------------------------------------------------------
% FINAL MODEL 3B PLANT
% ---------------------------------------------------------------

ms = 300;
mu = 40;
ks = 22000;
cs = 1500;
kt = 200000;

A = [0,        1,             0,               0;
    -ks/ms,   -cs/ms,         ks/ms,           cs/ms;
     0,        0,             0,               1;
     ks/mu,    cs/mu,        -(ks+kt)/mu,     -cs/mu];

B = [0; 1/ms; 0; -1/mu];
E = [0; 0; 0; kt/mu];
x0 = [0; 0; 0; 0];

%% ---------------------------------------------------------------
% FINAL LQR CONTROLLER BANK
% ---------------------------------------------------------------

Ca = [-ks/ms, -cs/ms, ks/ms, cs/ms];
Da = 1/ms;
Cs = [1, 0, -1, 0];

controller_names = ["Balanced"; "Comfort"; "Low Force"; "Protection"];
a_ref_bank = [2.0; 1.2; 2.0; 3.0];
s_ref_bank = [0.020; 0.030; 0.020; 0.012];
F_ref_bank = [500; 800; 350; 300];

K_bank = zeros(4,4);
for j = 1:4
    wa = 1/a_ref_bank(j)^2;
    ws = 1/s_ref_bank(j)^2;
    wF = 1/F_ref_bank(j)^2;
    Q = wa*(Ca'*Ca) + ws*(Cs'*Cs);
    N = wa*(Ca'*Da);
    R = wa*(Da^2) + wF;
    K_bank(j,:) = lqr(A,B,Q,R,N);
end

%% ---------------------------------------------------------------
% FINAL LOCKED MODEL 3B SCHEDULER / HARDWARE SETTINGS
% ---------------------------------------------------------------

Ts = 0.002;

window_samples = round(0.25/Ts);
fast_window_samples = round(0.05/Ts);
protection_hold_samples = round(0.060/Ts);
shock_hold_samples = round(0.40/Ts);

Body_Low_Threshold = 0.30;
Travel_Low_Threshold = 0.003;
Wheel_LowForce_Max = 15.0;
Wheel_Comfort_Max = 3.0;
Travel_Comfort_Min = 0.004;
Wheel_High_Threshold = 20.0;
Wheel_Fast_Protection_Threshold = 15.0;
Shock_RelVel_Threshold = 0.88;
Shock_Confirm_RelVel_Threshold = 0.80;
Shock_Confirm_Samples = 3;

Fmax_locked = 750;
Fdotmax_locked = 50000;

noise_free = struct( ...
    'bodyAccelStd',0, ...
    'wheelAccelStd',0, ...
    'travelStd',0, ...
    'relVelStd',0);

SensorDelaySamples_locked = 2;   % 4 ms at 500 Hz
SensorUpdateSamples_locked = 1;  % 500 Hz

%% ---------------------------------------------------------------
% SHOWCASE COMPOSITE + MULTI-SINE ROAD SIMULATION
% ---------------------------------------------------------------

road_length = 100;              % [m] -> 10 s at 10 m/s
v = wanted_speed;

% IMPORTANT: this exact road is supplied to the Model 3B plant and is
% also drawn in the animation. The visualization is therefore physically
% consistent with the controller response shown on screen.
zr_showcase = @(x) showcaseCompositeRoad(x);

fprintf('Simulating %s at %g m/s using locked Model 3B...\n', ...
    char(showcase_name),wanted_speed);

simAnim = simulateResponseScheduler( ...
    v, ...
    road_length, ...
    zr_showcase, ...
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

fprintf('Animation case loaded: %s at %g m/s, %.2f s, %d samples.\n', ...
    char(showcase_name),wanted_speed,simAnim.t(end),numel(simAnim.t));

% Print the ACTUAL MATLAB mode usage for this showcase road.
mode_pct = 100*[ ...
    mean(simAnim.mode==1), ...
    mean(simAnim.mode==2), ...
    mean(simAnim.mode==3), ...
    mean(simAnim.mode==4)];

fprintf('Mode usage: Balanced %.1f%% | Comfort %.1f%% | Low Force %.1f%% | Protection %.1f%%\n', ...
    mode_pct(1),mode_pct(2),mode_pct(3),mode_pct(4));

% Also report the controller usage specifically in the post-bump showcase
% and in the multi-sine region, so it is easy to verify that the ending
% actually exercises more than the Balanced controller.
idx_post_bump = simAnim.road_position >= 59.2;
idx_multisine = simAnim.road_position >= 68;

post_pct = 100*[ ...
    mean(simAnim.mode(idx_post_bump)==1), ...
    mean(simAnim.mode(idx_post_bump)==2), ...
    mean(simAnim.mode(idx_post_bump)==3), ...
    mean(simAnim.mode(idx_post_bump)==4)];

multi_pct = 100*[ ...
    mean(simAnim.mode(idx_multisine)==1), ...
    mean(simAnim.mode(idx_multisine)==2), ...
    mean(simAnim.mode(idx_multisine)==3), ...
    mean(simAnim.mode(idx_multisine)==4)];

fprintf('Post-bump usage: Balanced %.1f%% | Comfort %.1f%% | Low Force %.1f%% | Protection %.1f%%\n', ...
    post_pct(1),post_pct(2),post_pct(3),post_pct(4));

fprintf('Multi-sine usage: Balanced %.1f%% | Comfort %.1f%% | Low Force %.1f%% | Protection %.1f%%\n', ...
    multi_pct(1),multi_pct(2),multi_pct(3),multi_pct(4));

%% ---------------------------------------------------------------
% ANIMATION SETTINGS
% ---------------------------------------------------------------

record_video = true;
video_filename = sprintf('Model3B_CompositeMultiMode_%gms_10s_Animation.mp4',wanted_speed);

fps = 60;
playback_speed = 0.7;     % 10 s simulation -> approximately 10 s video

% Visual exaggeration of VEHICLE vertical motion.
vertical_scale = 2;

% ROAD exaggeration is intentionally limited to 2x.
% Vehicle relative motion remains at 5x so suspension motion is readable,
% while the road itself is only doubled to avoid an over-dramatic profile.
road_vertical_scale = 2;

% Keep the road contact point vertically anchored to y = 0.
% This removes common-mode vertical camera motion caused by road elevation
% while preserving the true relative suspension and tire motions.
anchor_road_vertically = true;

% Physical road span displayed across the screen [m].
% The car stays centered while this spatial window scrolls underneath.
road_half_window_m = 6;

% Clean 16:9 output.
fig = figure( ...
    'Units','pixels', ...
    'Position',[50 50 1920 1080], ...
    'Color','w', ...
    'Renderer','opengl', ...
    'MenuBar','none', ...
    'ToolBar','none', ...
    'NumberTitle','off', ...
    'Resize','off');

% IMPORTANT FOR A STABLE VIDEO:
% Use one fixed axes for the mechanical animation and a completely
% separate normalized overlay axes for the HUD text. This prevents the
% title / labels from moving when the vehicle geometry changes.
main_axes_position = [0.035 0.055 0.93 0.855];
axMain = axes( ...
    'Parent',fig, ...
    'Units','normalized', ...
    'Position',main_axes_position, ...
    'Color','none');

axes(axMain);

%% ---------------------------------------------------------------
% VIDEO WRITER
% ---------------------------------------------------------------

% Recover cleanly if a previous animation run stopped with an error while
% the MP4 file was still open.
if exist('videoObj','var')
    try
        close(videoObj);
    catch
    end
    clear videoObj
end

if record_video
    % Remove an incomplete file left by a previous failed run.
    if isfile(video_filename)
        try
            delete(video_filename);
        catch
            error(['Cannot overwrite ',video_filename,'. Close any video player ' ...
                   'using the file, or delete/rename the old MP4, then run again.']);
        end
    end

    videoObj = VideoWriter(video_filename,'MPEG-4');
    videoObj.FrameRate = fps;
    videoObj.Quality = 95;
    open(videoObj);
end

%% ---------------------------------------------------------------
% INTERPOLATE LOCKED MODEL 3B RESULTS TO VIDEO FRAME TIMES
% ---------------------------------------------------------------

t_end_anim = simAnim.t(end);
t_anim = (0:playback_speed/fps:t_end_anim)';

zs_anim = interp1(simAnim.t,simAnim.x(:,1),t_anim,'pchip');
zu_anim = interp1(simAnim.t,simAnim.x(:,3),t_anim,'pchip');

road_pos_anim = interp1( ...
    simAnim.t,simAnim.road_position,t_anim,'linear');

zr_anim = interp1( ...
    simAnim.t,simAnim.road,t_anim,'pchip');

Fa_anim = interp1( ...
    simAnim.t,simAnim.Fa,t_anim,'linear');

% Controller selection is discrete: nearest-neighbor interpolation avoids
% artificial fractional modes between switching instants.
mode_anim = interp1( ...
    simAnim.t,double(simAnim.mode),t_anim,'nearest');
mode_anim = round(mode_anim);

if isfield(simAnim,'Saturated')
    saturated_anim = interp1( ...
        simAnim.t,double(simAnim.Saturated),t_anim,'nearest') > 0.5;
else
    saturated_anim = false(size(t_anim));
end

%% ---------------------------------------------------------------
% CONTROLLER VISUAL IDENTITY
% ---------------------------------------------------------------

mode_names = [ ...
    "Balanced" ...
    "Comfort" ...
    "Low Force" ...
    "Protection" ...
    ];

% Mode colors: Balanced / Comfort / Low Force / Protection
mode_colors = [ ...
    0.0000 0.4470 0.7410; ...
    0.4660 0.6740 0.1880; ...
    0.5000 0.5000 0.5000; ...
    0.8500 0.3250 0.0980  ...
    ];

%% ---------------------------------------------------------------
% VEHICLE / DISPLAY GEOMETRY
% ---------------------------------------------------------------

% Plot-space road coordinates. These are DISPLAY coordinates, not meters.
xroad_plot = linspace(-0.90,0.90,1400);

wheel_radius = 0.055;

unsprung_width  = 0.24;
unsprung_height = 0.06;
wheel_to_unsprung_gap = 0.03;

% The dynamic states are deviations from static equilibrium. Because the
% animation exaggerates vertical motion, a fixed display gap can become
% negative during large compression. Choose the visual body baseline from
% the most compressed simulated suspension state so the spring/damper never
% geometrically invert. This changes only the drawing, not the simulation.
min_visual_suspension_gap = 0.050;
min_susp_travel = min(simAnim.x(:,1) - simAnim.x(:,3));
unsprung_top_at_zero = 2*wheel_radius + ...
    wheel_to_unsprung_gap + unsprung_height;

body_base = unsprung_top_at_zero + ...
    min_visual_suspension_gap - vertical_scale*min_susp_travel;

% Keep a sensible minimum baseline for mild simulations.
body_base = max(body_base,0.32);

body_width  = 0.50;
body_height = 0.08;

spring_x_center = -0.065;
spring_width = 0.018;
n_coils = 8;

damper_x = 0;
damper_width = 0.035;
piston_width = 0.025;

actuator_x = 0.065;
actuator_width = 0.035;

%% ---------------------------------------------------------------
% INITIAL FRAME GEOMETRY
% ---------------------------------------------------------------

i = 1;

x_car_m = road_pos_anim(i);

% Map the display window to physical road coordinates around the vehicle.
world_x_m = x_car_m + ...
    (xroad_plot/0.90)*road_half_window_m;

% Use the exact simulated road data. Outside [0,road_length], draw zero.
zroad = interp1( ...
    simAnim.road_position,simAnim.road,world_x_m,'linear',0);

zr_under_wheel = zr_anim(i);

tire_def_anim = zu_anim(i) - zr_under_wheel;

if anchor_road_vertically
    % Use the instantaneous road height below the wheel as the vertical
    % display reference. The road contact point therefore stays at y = 0.
    zroad_display = zroad - zr_under_wheel;
    wheel_center_y = wheel_radius + vertical_scale*tire_def_anim;
    zs_vis = body_base + vertical_scale*(zs_anim(i) - zr_under_wheel);
else
    zroad_display = zroad;
    wheel_center_y = ...
        vertical_scale*zr_under_wheel ...
        + wheel_radius ...
        + vertical_scale*tire_def_anim;
    zs_vis = body_base + vertical_scale*zs_anim(i);
end

unsprung_bottom = ...
    wheel_center_y + wheel_radius + wheel_to_unsprung_gap;

suspension_bottom = unsprung_bottom + unsprung_height;
suspension_top = zs_vis;
susp_height = suspension_top - suspension_bottom;

mode_i = max(1,min(4,mode_anim(i)));
mode_color = mode_colors(mode_i,:);

%% ---------------------------------------------------------------
% DRAW STATIC OBJECTS ONCE
% ---------------------------------------------------------------

axes(axMain);
hold(axMain,'on')

% ROAD
hRoad = plot( ...
    xroad_plot, ...
    road_vertical_scale*zroad_display, ...
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
    'FaceColor',[0.94 0.94 0.94], ...
    'EdgeColor','k', ...
    'LineWidth',2);

% UNSPRUNG MASS
hUnsprung = rectangle( ...
    'Position',[ ...
        -unsprung_width/2, ...
        unsprung_bottom, ...
        unsprung_width, ...
        unsprung_height], ...
    'FaceColor',[0.97 0.97 0.97], ...
    'EdgeColor','k', ...
    'LineWidth',2);

% WHEEL -> UNSPRUNG MASS CONNECTION
hWheelConnection = plot( ...
    [0 0], ...
    [wheel_center_y + wheel_radius, unsprung_bottom], ...
    'k', ...
    'LineWidth',2);

% SPRUNG MASS
hBody = rectangle( ...
    'Position',[ ...
        -body_width/2, ...
        zs_vis, ...
        body_width, ...
        body_height], ...
    'FaceColor',[0.97 0.97 0.97], ...
    'EdgeColor','k', ...
    'LineWidth',2.2);

% SPRING

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

% DAMPER

damper_body_bottom = ...
    suspension_bottom + 0.30*susp_height;

damper_body_top = ...
    suspension_bottom + 0.58*susp_height;

hDamperLower = plot( ...
    [damper_x damper_x], ...
    [suspension_bottom damper_body_bottom], ...
    'k', ...
    'LineWidth',2);

hDamperBody = rectangle( ...
    'Position',[ ...
        damper_x-damper_width/2, ...
        damper_body_bottom, ...
        damper_width, ...
        damper_body_top-damper_body_bottom], ...
    'FaceColor','w', ...
    'EdgeColor','k', ...
    'LineWidth',2);

hDamperUpper = plot( ...
    [damper_x damper_x], ...
    [damper_body_top suspension_top], ...
    'k', ...
    'LineWidth',2);

hDamperPiston = plot( ...
    [damper_x-piston_width damper_x+piston_width], ...
    [damper_body_top damper_body_top], ...
    'k', ...
    'LineWidth',2);

% ACTIVE ACTUATOR -- MODE COLORED

actuator_body_bottom = ...
    suspension_bottom + 0.25*susp_height;

actuator_body_top = ...
    suspension_bottom + 0.60*susp_height;

hActuatorLower = plot( ...
    [actuator_x actuator_x], ...
    [suspension_bottom actuator_body_bottom], ...
    'Color',mode_color, ...
    'LineWidth',3);

hActuatorBody = rectangle( ...
    'Position',[ ...
        actuator_x-actuator_width/2, ...
        actuator_body_bottom, ...
        actuator_width, ...
        actuator_body_top-actuator_body_bottom], ...
    'FaceColor',mode_color, ...
    'EdgeColor','k', ...
    'LineWidth',1.5);

hActuatorUpper = plot( ...
    [actuator_x actuator_x], ...
    [actuator_body_top suspension_top], ...
    'Color',mode_color, ...
    'LineWidth',3);

hActuatorPiston = plot( ...
    [actuator_x-0.015 actuator_x+0.015], ...
    [actuator_body_top actuator_body_top], ...
    'Color',mode_color, ...
    'LineWidth',3);

%% ---------------------------------------------------------------
% FIXED CAMERA + FIXED HEAD-UP DISPLAY
% ---------------------------------------------------------------

% Lock the mechanical axes once. Nothing in the animation loop changes
% these limits or the axes position.
xlim(axMain,[-0.90 0.90])
ylim(axMain,[-0.06 max(0.72,body_base + body_height + 0.16)])
axis(axMain,'equal')
axis(axMain,'manual')
axis(axMain,'off')
set(axMain,'Position',main_axes_position);

% HUD lives on its own full-figure overlay axes using normalized
% coordinates. The text therefore stays at exactly the same screen
% location regardless of road / suspension motion.
axHUD = axes( ...
    'Parent',fig, ...
    'Units','normalized', ...
    'Position',[0 0 1 1], ...
    'Color','none', ...
    'XLim',[0 1], ...
    'YLim',[0 1], ...
    'Visible','off', ...
    'HitTest','off');

hTitle = text( ...
    axHUD,0.50,0.955, ...
    sprintf('Model 3B - Showcase Multi-Sine Road, %g m/s',wanted_speed), ...
    'Units','normalized', ...
    'FontSize',20, ...
    'FontWeight','bold', ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment','top', ...
    'Clipping','off');

hModeText = text( ...
    axHUD,0.045,0.900, ...
    sprintf('Active controller: K_%d - %s', ...
    mode_i,mode_names(mode_i)), ...
    'Units','normalized', ...
    'FontSize',17, ...
    'FontWeight','bold', ...
    'Color',mode_color, ...
    'HorizontalAlignment','left', ...
    'VerticalAlignment','top', ...
    'Clipping','off');

hForceText = text( ...
    axHUD,0.045,0.855, ...
    sprintf('Actuator force: %+5.0f N',Fa_anim(i)), ...
    'Units','normalized', ...
    'FontSize',14, ...
    'HorizontalAlignment','left', ...
    'VerticalAlignment','top', ...
    'Clipping','off');

hPositionText = text( ...
    axHUD,0.955,0.900, ...
    sprintf('x = %.1f m    t = %.2f s', ...
    road_pos_anim(i),t_anim(i)), ...
    'Units','normalized', ...
    'FontSize',13, ...
    'HorizontalAlignment','right', ...
    'VerticalAlignment','top', ...
    'Clipping','off');

hScaleText = text( ...
    axHUD,0.045,0.035, ...
    sprintf('Vehicle motion exaggerated %gx; road display %gx; road contact anchored', ...
    vertical_scale,road_vertical_scale), ...
    'Units','normalized', ...
    'FontSize',10, ...
    'Color',[0.35 0.35 0.35], ...
    'HorizontalAlignment','left', ...
    'VerticalAlignment','bottom', ...
    'Clipping','off');

% Return focus to the mechanical axes before the animation begins.
axes(axMain);

%% ---------------------------------------------------------------
% AUTOMATIC REPORT / LINKEDIN SNAPSHOT
% ---------------------------------------------------------------

snapshot_target_x = 82;                % [m] near the bump for a dynamic teaser frame
snapshot_filename = 'Model3B_teaser.png';
snapshot_saved = false;

%% ---------------------------------------------------------------
% ANIMATION / VIDEO LOOP
% ---------------------------------------------------------------

for i = 1:length(t_anim)

    % ROAD
    x_car_m = road_pos_anim(i);

    world_x_m = x_car_m + ...
        (xroad_plot/0.90)*road_half_window_m;

    zroad = interp1( ...
        simAnim.road_position,simAnim.road,world_x_m,'linear',0);

    % VEHICLE POSITIONS
    zr_under_wheel = zr_anim(i);
    tire_def_anim = zu_anim(i) - zr_under_wheel;

    if anchor_road_vertically
        % Vertically stabilize the view at the tire-road contact location.
        % Only a display reference is changed; the simulated states remain
        % untouched and all relative deformations are preserved.
        zroad_display = zroad - zr_under_wheel;
        wheel_center_y = wheel_radius + vertical_scale*tire_def_anim;
        zs_vis = body_base + vertical_scale*(zs_anim(i) - zr_under_wheel);
    else
        zroad_display = zroad;
        wheel_center_y = ...
            vertical_scale*zr_under_wheel ...
            + wheel_radius ...
            + vertical_scale*tire_def_anim;
        zs_vis = body_base + vertical_scale*zs_anim(i);
    end

    set(hRoad,'YData',road_vertical_scale*zroad_display);

    unsprung_bottom = ...
        wheel_center_y + wheel_radius + wheel_to_unsprung_gap;

    set(hWheel, ...
        'Position',[ ...
        -wheel_radius, ...
        wheel_center_y-wheel_radius, ...
        2*wheel_radius, ...
        2*wheel_radius]);

    set(hUnsprung, ...
        'Position',[ ...
        -unsprung_width/2, ...
        unsprung_bottom, ...
        unsprung_width, ...
        unsprung_height]);

    set(hWheelConnection, ...
        'YData',[ ...
        wheel_center_y + wheel_radius, ...
        unsprung_bottom]);

    set(hBody, ...
        'Position',[ ...
        -body_width/2, ...
        zs_vis, ...
        body_width, ...
        body_height]);

    % SUSPENSION GEOMETRY
    suspension_bottom = ...
        unsprung_bottom + unsprung_height;

    suspension_top = zs_vis;
    susp_height = suspension_top - suspension_bottom;

    y_spring = linspace( ...
        suspension_bottom, ...
        suspension_top, ...
        2*n_coils + 1);

    set(hSpring,'YData',y_spring);

    % DAMPER
    damper_body_bottom = ...
        suspension_bottom + 0.30*susp_height;

    damper_body_top = ...
        suspension_bottom + 0.58*susp_height;

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

    % ACTIVE ACTUATOR
    actuator_body_bottom = ...
        suspension_bottom + 0.25*susp_height;

    actuator_body_top = ...
        suspension_bottom + 0.60*susp_height;

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

    % MODE COLOR + TEXT
    mode_i = max(1,min(4,mode_anim(i)));
    mode_color = mode_colors(mode_i,:);

    set(hActuatorLower, ...
        'Color',mode_color);

    set(hActuatorBody, ...
        'FaceColor',mode_color);

    set(hActuatorUpper, ...
        'Color',mode_color);

    set(hActuatorPiston, ...
        'Color',mode_color);

    set(hModeText, ...
        'String',sprintf( ...
        'Active controller: K_%d - %s', ...
        mode_i,mode_names(mode_i)), ...
        'Color',mode_color);

    if saturated_anim(i)
        force_string = sprintf( ...
            'Actuator force: %+5.0f N   [SATURATION]', ...
            Fa_anim(i));
    else
        force_string = sprintf( ...
            'Actuator force: %+5.0f N', ...
            Fa_anim(i));
    end

    set(hForceText,'String',force_string);

    set(hPositionText, ...
        'String',sprintf( ...
        'x = %.1f m    t = %.2f s', ...
        road_pos_anim(i),t_anim(i)));

    % Reassert the fixed camera box. This is intentionally redundant:
    % it prevents MATLAB graphics from altering the plot box during
    % long animation/video runs.
    set(axMain,'Position',main_axes_position);
    xlim(axMain,[-0.90 0.90]);
    ylim(axMain,[-0.06 max(0.72,body_base + body_height + 0.16)]);

    drawnow

    % AUTOMATIC SNAPSHOT
    % Save the first rendered frame at or beyond the selected road position.
    % exportgraphics captures the mechanical view and the fixed HUD together.
    if ~snapshot_saved && road_pos_anim(i) >= snapshot_target_x

        exportgraphics(fig, ...
            snapshot_filename, ...
            'Resolution',300);

        snapshot_saved = true;

        fprintf('Snapshot saved as: %s (x = %.2f m)\n', ...
            snapshot_filename,road_pos_anim(i));

    end

    % VIDEO FRAME
    if record_video
        frame = getframe(fig);

        % Windows display scaling can occasionally make getframe() return
        % a frame a few pixels away from the requested Full-HD size.
        % Normalize every frame to exactly 1920 x 1080 before writing.
        target_h = 1080;
        target_w = 1920;
        img = frame.cdata;

        if size(img,1) ~= target_h || size(img,2) ~= target_w
            if exist('imresize','file') == 2
                img = imresize(img,[target_h target_w]);
            else
                % Toolbox-free fallback: crop/pad to the requested size.
                fixed = uint8(255*ones(target_h,target_w,3));
                hh = min(target_h,size(img,1));
                ww = min(target_w,size(img,2));
                fixed(1:hh,1:ww,:) = img(1:hh,1:ww,:);
                img = fixed;
            end
        end

        frame.cdata = img;
        frame.colormap = [];
        writeVideo(videoObj,frame);
    end

end

if record_video
    close(videoObj);
    fprintf('Video saved as: %s\n',video_filename);
end

%% SNAPSHOT OUTPUT
% The script automatically saves one 300-dpi PNG at snapshot_target_x.
% Change snapshot_target_x near the animation-loop settings if you want
% a different frame.

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




function zr = showcaseCompositeRoad(x)

% COMPOSITE + MULTI-MODE MULTI-SINE SHOWCASE ROAD
% ------------------------------------------------
% The original benchmark road is retained through the bump. After the bump,
% a mild recovery is followed by three smoothly blended multi-sine regimes:
%
%   body-dominated  -> intended to make Comfort plausible,
%   mixed           -> intended to retain Balanced operation,
%   wheel-active    -> intended to make Protection plausible.
%
% IMPORTANT: nothing in the controller or scheduler is changed. Only the
% visualization road input is shaped so the locked controller bank can show
% a wider range of its natural switching behavior.

zr = zeros(size(x));

% ---------------------------------------------------------------
% 0--16 m: original mild section
% ---------------------------------------------------------------
idx = (x >= 0) & (x < 16);
zr(idx) = 0.002*sin(2*pi*x(idx)/8);

% ---------------------------------------------------------------
% 16--32 m: original long-wave section
% ---------------------------------------------------------------
idx = (x >= 16) & (x < 32);
zr(idx) = 0.015*sin(2*pi*(x(idx)-16)/8);

% ---------------------------------------------------------------
% 32--44 m: original medium-frequency section
% ---------------------------------------------------------------
idx = (x >= 32) & (x < 44);
zr(idx) = 0.010*sin(2*pi*(x(idx)-32)/2);

% ---------------------------------------------------------------
% 44--54 m: original high-frequency section
% ---------------------------------------------------------------
idx = (x >= 44) & (x < 54);
zr(idx) = 0.005*sin(2*pi*(x(idx)-44)/1);

% 54--58 m remains flat, matching the original bump approach.

% ---------------------------------------------------------------
% 58--59.2 m: original 30 mm raised-cosine bump
% ---------------------------------------------------------------
idx = (x >= 58) & (x <= 59.2);
zr(idx) = (0.03/2) .* ...
    (1-cos(2*pi*(x(idx)-58)/1.2));

% ---------------------------------------------------------------
% 59.2--68 m: mild smooth recovery
% ---------------------------------------------------------------
mild = 0.002*sin(2*pi*(x-59.2)/8);
idx = (x > 59.2) & (x < 68);
zr(idx) = mild(idx);

% ---------------------------------------------------------------
% MULTI-SINE REGIMES
% ---------------------------------------------------------------
% All three remain deterministic sums of spatial sinusoids. Their spectra
% are deliberately different so the measured vehicle response changes in a
% way that can naturally exercise different gains from the locked bank.

% A) Body-dominated multi-sine: long wavelengths, very little short-wave
%    content. At 10 m/s this primarily excites lower-frequency body motion.
xb = x - 68;
body_multi = ...
    0.0080*sin(2*pi*xb/12.0 + 0.20) ...
    + 0.0035*sin(2*pi*xb/6.0 + 1.05) ...
    + 0.0004*sin(2*pi*xb/2.8 + 1.80);

% B) Mixed multi-sine: medium spatial wavelengths / moderate amplitudes.
xm = x - 78;
mixed_multi = ...
    0.0045*sin(2*pi*xm/6.2 + 0.35) ...
    + 0.0025*sin(2*pi*xm/2.4 + 1.20) ...
    + 0.0011*sin(2*pi*xm/1.15 + 2.10);

% C) Wheel-active multi-sine: irregular short-wave content. This is not a
%    single-frequency wheel-hop signal; several high spatial frequencies
%    combine so it looks visibly different from the earlier 1 m sine.
xw = x - 88;
wheel_multi = ...
    0.0030*sin(2*pi*xw/1.35 + 0.30) ...
    + 0.0020*sin(2*pi*xw/0.82 + 1.25) ...
    + 0.0012*sin(2*pi*xw/0.58 + 2.20);

% ---------------------------------------------------------------
% 68--70 m: mild -> body-dominated multi-sine
% ---------------------------------------------------------------
idx = (x >= 68) & (x < 70);
w = 0.5*(1-cos(pi*(x(idx)-68)/2));
zr(idx) = (1-w).*mild(idx) + w.*body_multi(idx);

% 70--78 m: full body-dominated multi-sine
idx = (x >= 70) & (x < 78);
zr(idx) = body_multi(idx);

% ---------------------------------------------------------------
% 78--80 m: body-dominated -> mixed
% ---------------------------------------------------------------
idx = (x >= 78) & (x < 80);
w = 0.5*(1-cos(pi*(x(idx)-78)/2));
zr(idx) = (1-w).*body_multi(idx) + w.*mixed_multi(idx);

% 80--88 m: full mixed multi-sine
idx = (x >= 80) & (x < 88);
zr(idx) = mixed_multi(idx);

% ---------------------------------------------------------------
% 88--90 m: mixed -> wheel-active
% ---------------------------------------------------------------
idx = (x >= 88) & (x < 90);
w = 0.5*(1-cos(pi*(x(idx)-88)/2));
zr(idx) = (1-w).*mixed_multi(idx) + w.*wheel_multi(idx);

% 90--98 m: full wheel-active multi-sine
idx = (x >= 90) & (x < 98);
zr(idx) = wheel_multi(idx);

% 98--100 m: smooth fade to zero
idx = (x >= 98) & (x <= 100);
w = 0.5*(1+cos(pi*(x(idx)-98)/2));
zr(idx) = w.*wheel_multi(idx);

end
