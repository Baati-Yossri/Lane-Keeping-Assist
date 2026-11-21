
%% ROAD SCENARIO SELECTOR

% Choose the road:
% 1 = Constant-curvature road
% 2 = Winding S-shaped road
% 3 = Straight road

if ~exist('roadScenario','var')
    roadScenario = 1;
end
% Simulation duration and time resolution
StopTime = 10;
dt = 0.01;
tRoad = (0:dt:StopTime)';
% Vehicle speed
v = 30; % m/s

% Generate curvature according to selected scenario
switch roadScenario

    case 1
        % Constant curvature
        kappaValues = 0.005 * ones(size(tRoad));

    
case 2
    % Aggressive winding road with hairpin-like bends

    curvatureAmplitude = 0.085; %yosri remember more is tighter curves
    curvaturePeriod = 4; %yosri remember less is more curves
    %0.085 and 4 are the values in which i tested and got a lovely result
    %showing lots of curves, good handling and good correction 

    kappaValues = curvatureAmplitude * ...
        sin(2*pi*tRoad/curvaturePeriod);


    case 3
        % Straight road
        kappaValues = zeros(size(tRoad));

    otherwise
        error('Invalid road scenario selection.');
end

% Create a time-dependent signal for Simulink
kappaSignal = timeseries(kappaValues, tRoad);

disp('Road scenario generated successfully.');
