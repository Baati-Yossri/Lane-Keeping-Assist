
%% LKA ANIMATION - ALL ROAD SCENARIOS
% Supports:
% Scenario 1: Constant-curvature road
% Scenario 2: Winding road
% Scenario 3: Straight road
%
% Controller ON/OFF
% Disturbance ON/OFF
% Slow motion and highlighting during disturbance and recovery

clc;

%% 1. EXTRACT SIMULATION RESULTS

t = out.ey_out.Time(:);
ey = out.ey_out.Data(:);
ephi = out.ephi_out.Data(:);
delta = out.delta_out.Data(:);
u = out.u_out.Data(:);

%% 2. OBTAIN ROAD CURVATURE

kappa = interp1(kappaSignal.Time, ...
                kappaSignal.Data, t, 'linear', 'extrap');

%% 3. RECONSTRUCT REFERENCE ROAD

psiRoad = cumtrapz(t, v .* kappa);

Xroad = cumtrapz(t, v .* cos(psiRoad));
Yroad = cumtrapz(t, v .* sin(psiRoad));

%% 4. CALCULATE VEHICLE TRAJECTORY

psiVehicle = psiRoad + ephi;

X0 = Xroad(1) - ey(1)*sin(psiRoad(1));
Y0 = Yroad(1) + ey(1)*cos(psiRoad(1));

if controllerEnabled == 1

    Xvehicle = Xroad - ey.*sin(psiRoad);
    Yvehicle = Yroad + ey.*cos(psiRoad);

else

    initialHeading = psiVehicle(1);

    Xvehicle = X0 + v*t*cos(initialHeading);
    Yvehicle = Y0 + v*t*sin(initialHeading);

end

%% 5. ROAD BOUNDARIES

roadWidth = 3.6;
halfWidth = roadWidth/2;

Xleft = Xroad - halfWidth.*sin(psiRoad);
Yleft = Yroad + halfWidth.*cos(psiRoad);

Xright = Xroad + halfWidth.*sin(psiRoad);
Yright = Yroad - halfWidth.*cos(psiRoad);

disp('Road and vehicle trajectories reconstructed.');

%% 6. DISTURBANCE SETTINGS

if ~exist('disturbanceEnabled','var')
    disturbanceEnabled = 0;
end

if disturbanceEnabled == 1

    disturbanceStart = evalin('base','disturbanceStart');
    disturbanceEnd = evalin('base','disturbanceEnd');

    recoveryEnd = disturbanceEnd + 2.0;

    if exist('dySignal','var')
        dy = interp1(dySignal.Time, ...
                     dySignal.Data, t, 'previous', 0);
    else
        dy = zeros(size(t));
    end

    disturbanceActive = ...
        t >= disturbanceStart & t <= disturbanceEnd;

    highlightActive = ...
        t >= disturbanceStart & t <= recoveryEnd;

else

    disturbanceStart = 0;
    disturbanceEnd = 0;
    recoveryEnd = 0;

    dy = zeros(size(t));

    disturbanceActive = false(size(t));
    highlightActive = false(size(t));

end

%% 7. ANIMATION SETTINGS

carLength = 4.5;
carWidth = 1.8;

figure('Name','LKA - All Road Scenarios', ...
       'Color','w');

hold on;
grid on;
axis equal;

% Road boundaries
plot(Xleft,Yleft,'k','LineWidth',2);
plot(Xright,Yright,'k','LineWidth',2);

% Reference centerline
plot(Xroad,Yroad,'k--','LineWidth',1.2);

% Vehicle trajectory
trajectory = animatedline( ...
    'Color','b','LineWidth',1.8);

% Highlight trajectory during disturbance and recovery
disturbanceTrajectory = animatedline( ...
    'Color',[1 0.3 0],'LineWidth',2.8);

% Vehicle body
car = patch('XData',zeros(1,4), ...
            'YData',zeros(1,4), ...
            'FaceColor',[0.1 0.45 0.9], ...
            'EdgeColor','k');

% Vehicle front indicator
front = plot(nan,nan,'r','LineWidth',2);

% Lateral deviation indicator
deviationLine = plot(nan,nan,'r--','LineWidth',2);

% Information panel
info = text(0.98,0.96,'', ...
    'Units','normalized', ...
    'FontSize',11, ...
    'VerticalAlignment','top', ...
    'HorizontalAlignment','right', ...
    'BackgroundColor','w', ...
    'Margin',6);

xlabel('X position (m)');
ylabel('Y position (m)');

if controllerEnabled == 1
    controllerText = 'ON';
else
    controllerText = 'OFF';
end

if disturbanceEnabled == 1
    disturbanceText = 'ON';
else
    disturbanceText = 'OFF';
end

title(sprintf('LKA - Scenario %d | Controller %s | Disturbance %s', ...
      roadScenario,controllerText,disturbanceText));

% Animation limits
margin = 10;

xlim([min([Xroad;Xvehicle])-margin ...
      max([Xroad;Xvehicle])+margin]);

ylim([min([Yroad;Yvehicle])-margin ...
      max([Yroad;Yvehicle])+margin]);

%% 8. ANIMATION LOOP

for i = 1:length(t)

    % Vehicle center and orientation
    xc = Xvehicle(i);
    yc = Yvehicle(i);
    theta = psiVehicle(i);

    % Vehicle rectangle
    localX = [-carLength/2 carLength/2 ...
               carLength/2 -carLength/2];

    localY = [-carWidth/2 -carWidth/2 ...
               carWidth/2 carWidth/2];

    % Rotate vehicle
    rotatedX = xc + localX*cos(theta) - localY*sin(theta);
    rotatedY = yc + localX*sin(theta) + localY*cos(theta);

    % Vehicle color and highlighting
    if highlightActive(i)

        set(car,'FaceColor',[1 0.35 0.05]);

        addpoints(disturbanceTrajectory,xc,yc);

    else

        set(car,'FaceColor',[0.1 0.45 0.9]);

    end

    % Update vehicle body
    set(car,'XData',rotatedX,'YData',rotatedY);

    % Update front indicator
    set(front, ...
        'XData',[xc xc+(carLength/2)*cos(theta)], ...
        'YData',[yc yc+(carLength/2)*sin(theta)]);

    % Update normal trajectory
    addpoints(trajectory,xc,yc);

    % Draw lateral deviation
    set(deviationLine, ...
        'XData',[Xroad(i),Xvehicle(i)], ...
        'YData',[Yroad(i),Yvehicle(i)]);

    %% Disturbance status

    if disturbanceEnabled == 1

        if disturbanceActive(i)

            disturbanceStatus = 'DISTURBANCE ACTIVE';

        elseif t(i) > disturbanceEnd && t(i) <= recoveryEnd

            disturbanceStatus = 'RECOVERY PHASE';

        else

            disturbanceStatus = 'NORMAL OPERATION';

        end

    else

        disturbanceStatus = 'NORMAL OPERATION';

    end

    %% Update information panel

    if disturbanceEnabled == 1

        set(info,'String',sprintf( ...
            ['Time: %.2f s\n' ...
             'Controller: %s\n' ...
             'Disturbance: %s\n' ...
             'Lateral error: %.3f m\n' ...
             'Heading error: %.4f rad\n' ...
             'Steering: %.3f rad\n' ...
             'Lateral disturbance: %.2f m/s\n' ...
             'Status: %s'], ...
             t(i),controllerText,disturbanceText, ...
             ey(i),ephi(i),delta(i),dy(i),disturbanceStatus));

    else

        set(info,'String',sprintf( ...
            ['Time: %.2f s\n' ...
             'Controller: %s\n' ...
             'Disturbance: OFF\n' ...
             'Lateral error: %.3f m\n' ...
             'Heading error: %.4f rad\n' ...
             'Steering: %.3f rad\n' ...
             'Status: NORMAL OPERATION'], ...
             t(i),controllerText,ey(i),ephi(i), ...
             delta(i)));

    end

    drawnow;

    %% Animation speed

    if disturbanceEnabled == 1 && highlightActive(i)

        pause(0.15);  % Slow motion during disturbance and recovery

    else

        pause(0.03);  % Normal animation speed

    end

end

disp('LKA animation completed.');
