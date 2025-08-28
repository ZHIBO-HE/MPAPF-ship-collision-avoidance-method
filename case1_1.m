% CASE1_MPAPF_DEMO  
% A cleaned and commented demo script for:
%   "A novel model predictive artificial potential field based ship motion
%    planning method considering COLREGs for complex encounter scenarios — Case 1"
%
% Notes
% -----
% 1) This script keeps the original variable names (including globals) so it
%    remains compatible with your existing helper functions such as
%    `Copy_of_test4potential`, `shipdynamic`, `iscollisionavoidacne`,
%    `COLREGS_situation`, `domaindisplay`, `shipDisplay3`, `DCPA_TCPA`,
%    and the cost functions `PSO_MPC_Cost_Function`,
%    `MPC_Cost_Function_Targetship`.
% 2) UI input ("input(prompt)") has been replaced with a configurable default
%    at the top. Adjust `parameter.prediction_step` below as needed.
% 3) Added English comments, clarified units, grouped parameters, and reduced
%    magic numbers. Plot sections are optional and can be toggled.
% 4) Particle Swarm and fmincon options are set defensively. Ensure you have
%    the Optimization Toolbox and Global Optimization Toolbox available.
% 5) Randomness is seeded to make obstacle placement reproducible.
%
% -------------------------------------------------------------------------
% Housekeeping
% -------------------------------------------------------------------------
% Close figures and clear only what this script defines.  
% (We intentionally avoid `clear all` to preserve breakpoints.)
close all; clc;

% -------------------------------------------------------------------------
% Globals used across your existing helper functions (kept for compatibility)
% -------------------------------------------------------------------------
% WARNING: Refactoring away from globals is recommended long-term, but the
% dependent functions in your project likely rely on these names.

global mat_point        % static obstacle points (generated)
global end_point        % current end point (may be adjusted by COLREGs logic)
global ship             % own ship state (struct)
global ship1            % copy of own ship state (for constraints)
global parameter        % global parameter set (struct)
global current_step     % step index in main loop
global targetship       % array of target ships (struct array)
global tmp_end_point    % original end point (before adjustments)
global tsID             % index of current target ship in some routines

% Optional globals referenced elsewhere
% (Left defined for compatibility; not all are used here.)

global end_point_targetship
global start_point

% -------------------------------------------------------------------------
% Configuration (edit these as needed)
% -------------------------------------------------------------------------
parameter = struct();

% Environment / map
parameter.map_size        = [8, 4.5];     % [nm, nm] display area
parameter.map_min         = 0.05;         % grid resolution for potential field visualization
parameter.scale           = 1852;         % 1 simulation unit = 1 NM = 1852 m
parameter.time            = 5;            % [s] time per step (discrete simulation step)
parameter.searching_step  = 3000;         % max steps

% Hydrodynamics / current (speed in knots, converted to NM/s below if needed)
parameter.waterspeed      = 0/1852;       % [NM/s] (0 knots by default)
parameter.waterangle      = 45;           % [deg] current direction (0° = +Y axis)
parameter.water           = [sind(parameter.waterangle) cosd(parameter.waterangle)] * parameter.waterspeed;

% Potential field and obstacle thresholds
parameter.minpotential        = 0.001;
parameter.minpotential4ship   = 0.01;
parameter.minobstacle         = 0.03;
parameter.maxobstacle         = 0.2;
parameter.safeobstacle        = 5;        % [NM] safety distance to static obstacles
parameter.amplification       = 5;        % potential amplification factor

% Collision avoidance distances (COLREGs context)
parameter.safecv              = 2.5;      % [NM] safety distance for CV
parameter.dangercv            = 0.5;      % [NM] danger distance for CV
parameter.shipdomain          = 5;        % scaling factor for ship domain visualization

% Number of target ships
parameter.tsNum               = 1;

% COLREGs domain templates (Wang 2010) — scaled later by ship length/scale
parameter.situs1 = [6.1 3.9 3.25 1.75];  % head-on
parameter.situs2 = [6.1 3.9 3.25 1.75];  % crossing (give-way)
parameter.situs3 = [0.0 0.0 0.00 0.00];  % crossing (stand-on)
parameter.situs0 = [6.0 6.0 1.75 1.75];  % normal navigation

% Path endpoints (in map coordinates, NM)
start_point   = [1, 1];
end_point     = [7, 4];
tmp_end_point = end_point;  % store the original target

% Motion prediction horizon (replace previous `input(prompt)`)
parameter.prediction_step = 5;   % <-- set your desired horizon length here (1–10)

% End potential shaping term (depends on start/end distance)
parameter.endbeta = -log(parameter.minpotential) / ( (norm(end_point - start_point) * 2)^2 );

% Static obstacles — count per region and their bounding boxes
% static_obs_num  : [nRegion1; nRegion2]
% static_obs_area : [x_min, y_min, x_max, y_max] for each region (NM)
static_obs_num  = [12; 6];
static_obs_area = [0.5, 0.0, 4.5, 4.0;  ...
                   4.5, 2.0, 6.5, 3.5];

% Dynamic ships enable flags (1 = enabled per slot)
dynamic_ships = [1; 1; 1; 1]; %#ok<NASGU> % kept for compatibility if referenced elsewhere

% Own ship initial state and model parameters
ship = struct();
ship.speed      = 8.23;                     % [knots] (comment says 16 kn, but value=8.23)
ship.v          = 0;                         % surge velocity placeholder (if used elsewhere)
ship.data       = [ ...                      % hydrodynamic lookup table: [U K T n3]
    6   0.08 20 0.4;
    9   0.18 27 0.6;
    12  0.23 21 0.3
];

% Interpolate dynamic model parameters as functions of speed U
ship.k    = interp1(ship.data(:,1), ship.data(:,2), ship.speed, 'linear', 'extrap');
ship.T    = interp1(ship.data(:,1), ship.data(:,3), ship.speed, 'linear', 'extrap');
ship.n3   = interp1(ship.data(:,1), ship.data(:,4), ship.speed, 'linear', 'extrap');

% Actuator & geometry constraints
ship.Ddelta   = 10 * pi/180;  % [rad/s] max rudder rate
ship.delta    = 30 * pi/180;  % [rad]   max rudder angle
ship.length   = 100;          % [m]     ship length (for domain scaling)
ship.p_leader = -8;           % leader parameters (if used)
ship.alpha_leader = 3;        % leader parameters (if used)

% Initial kinematic state
ship.yaw        = 45;         % [deg] course angle
ship.yaw_rate   = 0;          % [deg/s] or [rad/s] per your dynamics (consistent with helpers)
ship.rudder     = 0;          % [rad]
ship.rudder_rate= 0;          % [rad/s]
ship.position   = start_point;% [NM, NM]
ship.gamma      = 1;          % cost weight (if used in cost function)
ship.lamda      = log(1/parameter.minpotential4ship - 1) / ((parameter.shipdomain)^2 - 1);
ship.prediction_postion = []; % (kept as-is)

ship1 = ship;                 % duplicate for follower constraints if needed

% -------------------------------------------------------------------------
% Derived quantities & reproducibility
% -------------------------------------------------------------------------
rng(1); % seed randomness for obstacle generation
mat_point = init_obstacles(static_obs_num, static_obs_area); % generate static obstacles

% Decision variable bounds for PSO (rudder commands over horizon)
LB_follower = zeros(1, 2*parameter.prediction_step);
UB_follower = zeros(1, 2*parameter.prediction_step);
for ii = 1:parameter.prediction_step
    LB_follower(2*ii-1:2*ii) = [0, -ship.delta];   % [placeholder_control, rudder_lower]
    UB_follower(2*ii-1:2*ii) = [0,  ship.delta];   % [placeholder_control, rudder_upper]
end
parameter.navars = 2 * parameter.prediction_step;  % dimension of decision vector

% Target ships initialization (helper sets up array)
targetship = init_ship(ship, 'others', 1000); % at least 1 target ship is created

% End-of-path tolerance (NM)
parameter.end1 = 0.05;

% Visualization toggles
SHOW_POTENTIAL  = true;   % heavy: draws 3D potential mesh
SHOW_RUNTIME    = true;    % prints elapsed time at end

% -------------------------------------------------------------------------
% Optional: visualize static potential field (expensive)
% -------------------------------------------------------------------------
if SHOW_POTENTIAL
    figure(1);
    % Compute potential map z(x,y) by sampling grid
    x = 0:parameter.map_min:parameter.map_size(1);
    y = 0:parameter.map_min:parameter.map_size(2);
    z = zeros(numel(x), numel(y));
    for ix = 1:numel(x)
        for iy = 1:numel(y)
            p = [x(ix), y(iy)];
            z(ix, iy) = Copy_of_test4potential(p);
        end
    end
    [X, Y] = meshgrid(x, y);
    mesh(X', Y', z); colorbar; hold on;
    axis square; axis equal;
    set(gcf, 'position', [1 1 1900 1000]);
    pause(1);
    close(1);
end

% -------------------------------------------------------------------------
% Main display figure
% -------------------------------------------------------------------------
draw2();  % external helper for background map/axes
set(gcf, 'position', [200 200 1361/1.5 750/2]);
hold on;

% Prepare storage for plotting handles and logs
hline1 = gobjects(1);
hshipd = gobjects(1);
hshipt = gobjects(1);
hdomain = gobjects(1);
hdomain1 = gobjects(1);

path1 = [];         % [x y potential speed yaw yaw_rate rudder_deg]
tsPath = cell(parameter.tsNum, 1);
tsPath(:) = {[]};
leader_stop = 0;    %#ok<NASGU>

% Precompute a default domain scale for the first target (if present)
R_tmp = parameter.situs2 * ship.length / parameter.scale; %#ok<NASGU>

% -------------------------------------------------------------------------
% Optimization options
% -------------------------------------------------------------------------
try
    psoOpts = optimoptions('particleswarm', ...
        'Display','none', 'MaxIterations', 50, 'SwarmSize', 50);
catch
    % Fallback if options creation fails (older MATLAB)
    psoOpts = struct();
end

try
    fminOpts = optimoptions(@fmincon, 'Algorithm','active-set', ...
        'MaxFunctionEvaluations', 500, 'Display','none');
catch
    fminOpts = [];
end

% -------------------------------------------------------------------------
% Main simulation loop
% -------------------------------------------------------------------------
tic;  % start timing

for current_step = 1:parameter.searching_step
    pause(0.02); % small delay for visualization smoothness

    % Heuristic stability check of course angle between previous two steps
    steable = 0; %#ok<NASGU> % spelled as in original code
    if current_step >= 3
        d_prev = path1(current_step-1,5) - path1(current_step-2,5);
        if d_prev <= 1 && d_prev >= -1
            steable = 1;
        end
    end

    % Ensure targetship exists
    if ~exist('targetship','var') || isempty(targetship)
        targetship = init_ship(ship, 'others', 1000);
    end

    % Collision-avoidance situation assessment
    [iscv, cvship] = iscollisionavoidacne(ship, targetship); %#ok<ASGLU>

    % Adjust end point for give-way/head-on situations to bias motion
    if iscv && (targetship(cvship).situation == 2 || targetship(cvship).situation == 1)
        tmp_l     = targetship(cvship).shipdomain(2) * parameter.shipdomain;
        end_point = targetship(cvship).position - [sind(targetship(cvship).yaw), cosd(targetship(cvship).yaw)] * tmp_l;
    else
        end_point = tmp_end_point;
    end

    % Update COLREGs situation for other targets when stable
    for tship = 2:numel(targetship)
        if steable && targetship(tship).situation == 0
            targetship(tship).situation = COLREGS_situation(ship, targetship(tship));
            switch targetship(tship).situation
                case 1 % head-on
                    targetship(tship).shipdomain = parameter.situs1 * targetship(tship).length / parameter.scale;
                case 2 % crossing give-way
                    targetship(tship).shipdomain = parameter.situs2 * targetship(tship).length / parameter.scale;
                case 3 % crossing stand-on
                    targetship(tship).shipdomain = parameter.situs3 * targetship(tship).length / parameter.scale;
                otherwise % normal
                    targetship(tship).shipdomain = parameter.situs0 * targetship(tship).length / parameter.scale;
            end
        end
    end

    % Predict future positions of target ships over the horizon
    for pship = 1:parameter.prediction_step
        for tship = 1:numel(targetship)
            tmp_tship = shipdynamic(targetship(tship), 0, parameter.prediction_step * parameter.time);
            targetship(tship).predicted_position(pship, :) = tmp_tship.position; %#ok<SAGROW>
        end
    end

    % Solve own-ship control over the horizon via Particle Swarm
    if isempty(psoOpts)
        % Minimal call without options if creation failed
        result_mat = particleswarm(@PSO_MPC_Cost_Function, parameter.navars, LB_follower, UB_follower);
    else
        result_mat = particleswarm(@PSO_MPC_Cost_Function, parameter.navars, LB_follower, UB_follower, psoOpts);
    end

    % Roll out predicted own-ship trajectory with the optimized inputs
    tmp_tship = ship;
    for pship = 1:parameter.prediction_step
        tmp_tship = shipdynamic(tmp_tship, result_mat(2*pship));
        ship.predicted_position(pship, :) = tmp_tship.position; %#ok<SAGROW>
    end

    % Apply the first control (receding horizon)
    ship = shipdynamic(ship, result_mat(2));

    % Check for arrival
    if norm(ship.position - end_point) <= parameter.end1
        break; % reached goal
    end

    % ---------------------------------------------------------------------
    % Visualization (remove for faster runs)
    % ---------------------------------------------------------------------
    try
        if current_step >= 2
            if isgraphics(hline1), delete(hline1); end
            if isgraphics(hshipd), delete(hshipd); end
        end
        if isgraphics(hshipt), delete(hshipt); end
        if isgraphics(hdomain), delete(hdomain); end
        if isgraphics(hdomain1), delete(hdomain1); end

        subplot('Position', [0.0686204431736955 0.348877374784111 0.429433722500199 0.583657167530225]);
        box on;
        path1(current_step, :) = [ ...
            ship.position, ...                                        % x, y
            Copy_of_test4potential(ship.position), ...                % potential
            ship.speed, ...                                           % speed (knots)
            mod(ship.yaw, 360), ...                                   % course angle (deg)
            ship.yaw_rate, ...                                        % yaw rate
            ship.rudder * 180/pi ...                                  % rudder (deg)
        ]; %#ok<SAGROW>
        hline1 = plot(path1(:,1), path1(:,2), 'b-', 'LineWidth', 1);
        hshipd = shipDisplay3([ship.yaw 0 0], path1(current_step,1), path1(current_step,2), 0, 0.3, [0 1 0]);

        if numel(targetship) > 1
            hshipt  = shipDisplay3([targetship(2).yaw 0 0], targetship(2).position(1), targetship(2).position(2), 0, 0.1, [1 0 0]);
            R_tmp   = parameter.situs2 * targetship(2).length / parameter.scale;
            hdomain  = domaindisplay(targetship(2).position(1), targetship(2).position(2), R_tmp, targetship(2).yaw, [1 1 1]);
            hdomain1 = domaindisplay(targetship(2).position(1), targetship(2).position(2), R_tmp*parameter.shipdomain, targetship(2).yaw, [1 1 1]);
            if ~isempty(tsPath{2})
                plot(tsPath{2}(:,1), tsPath{2}(:,2), 'r-', 'LineWidth', 1);
            end
        end

        legend1 = legend('start', 'target', 'static obstacles', 'detection area', 'path by own algorithm', 'own ship');
        set(legend1, 'Position', [0.3227 0.2324 0.1796 0.2522]);

        subplot('Position', [0.5536 0.7392 0.3345 0.1952]);
        plot(1:5:5*current_step, path1(1:current_step, 7), 'b--', 'LineWidth', 1);
        xlabel('time (s)'); ylabel('rudder (deg)');
        axis([0 5*current_step -35 35]); set(gca, 'FontSize', 10);

        subplot('Position', [0.5547 0.3497 0.3338 0.2798]);
        plot(1:5:5*current_step, path1(1:current_step, 5), 'b-.', 'LineWidth', 1);
        axis([0 5*current_step 0 360]); xlabel('time (s)'); ylabel('course angle (deg)');

        % GIF export (optional)
        F = getframe(gcf); I = frame2im(F); [I,map] = rgb2ind(I,256);
        if current_step == 1
            imwrite(I, map, 'case1_demo.gif', 'gif', 'Loopcount', inf, 'DelayTime', 0.2);
        elseif mod(current_step,5) == 1
            imwrite(I, map, 'case1_demo.gif', 'gif', 'WriteMode', 'append', 'DelayTime', 0.2);
        end
    catch vizErr
        warning('Visualization error at step %d: %s', current_step, vizErr.message);
    end

    % ---------------------------------------------------------------------
    % Target ship receding-horizon update (for tship >= 2)
    % ---------------------------------------------------------------------
    for tship = 2:numel(targetship)
        tsID = tship; %#ok<NASGU>
        end_point_targetship = [1, 1]; %#ok<NASGU> % placeholder used by cost function

        if isempty(psoOpts)
            result_mat_ts = particleswarm(@MPC_Cost_Function_Targetship, parameter.navars, LB_follower, UB_follower);
        else
            result_mat_ts = particleswarm(@MPC_Cost_Function_Targetship, parameter.navars, LB_follower, UB_follower, psoOpts);
        end
        targetship(tship) = shipdynamic(targetship(tship), result_mat_ts(2));

        tsPath{tship} = [tsPath{tship}; ...
            targetship(tship).position, ...
            Copy_of_test4potential(targetship(tship).position), ...
            targetship(tship).speed, ...
            targetship(tship).yaw, ...
            targetship(tship).yaw_rate, ...
            targetship(tship).rudder * 180/pi ...
        ];

        % DCPA/TCPA diagnostics (could be logged if desired)
        [dcpa, tcpa] = DCPA_TCPA(ship, targetship(tship)); %#ok<NASGU>
    end
end

if SHOW_RUNTIME
    elapsed = toc; %#ok<TNMLP>
    fprintf('Simulation finished in %.2f s (steps executed: %d)\n', elapsed, current_step);
else
    toc; %#ok<TNMLP>
end

% End of script.
