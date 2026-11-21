
function run_LKA(scenario, controllerState)

%% LKA MASTER FUNCTION
% Road scenarios:
% 1 = Constant-curvature road
% 2 = Winding road
% 3 = Straight road
%
% Controller:
% 0 = OFF
% 1 = ON
%
% Examples:
% run_LKA(2,0)
% run_LKA(2,1)

    clc;

    %% Validate inputs

    if nargin < 1
        error('Select a road: run_LKA(1), run_LKA(2), or run_LKA(3).');
    end

    if nargin < 2
        controllerState = 1; % Default: controller ON
    end

    if ~ismember(scenario,[1 2 3])
        error('Invalid road scenario. Choose 1, 2, or 3.');
    end

    if ~ismember(controllerState,[0 1])
        error('Controller state must be 0 (OFF) or 1 (ON).');
    end

    %% Display configuration

    fprintf('\n=================================\n');
    fprintf('          LKA SIMULATION\n');
    fprintf('=================================\n');

    fprintf('Road scenario: %d\n',scenario);

    if controllerState == 1
        fprintf('Controller: ON\n');
    else
        fprintf('Controller: OFF\n');
    end

    %% Configure MATLAB workspace

    assignin('base','roadScenario',scenario);
    assignin('base','controllerEnabled',controllerState);

    %% Generate road curvature

    evalin('base','RoadScenario');

    %% Configure Simulink

    model = 'LKA_Model';

    load_system(model);

    StopTime = evalin('base','StopTime');

    set_param(model,'StopTime',num2str(StopTime));

    %% Run simulation

    fprintf('\nRunning simulation...\n');

    simOut = sim(model);

    assignin('base','out',simOut);
    assignin('base','controllerEnabled',controllerState);


    fprintf('Simulation completed successfully.\n');

    %% Launch animation

    fprintf('Opening animation...\n');

    evalin('base','LKA_Animation_AllRoads');

    fprintf('\n=================================\n');
    fprintf('       SIMULATION FINISHED\n');
    fprintf('=================================\n');

end
