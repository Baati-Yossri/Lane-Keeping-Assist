
%% LKA ANIMATION - ALL ROAD SCENARIOS
% Scenario 1: Constant-curvature road
% Scenario 2: Winding road
% Scenario 3: Straight road
%
% Controller ON/OFF
% Disturbance ON/OFF
% Slow motion and highlighting during disturbance and recovery
% Animated steering wheel synchronized with steering angle

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

%% 7. MAIN ANIMATION WINDOW

carLength = 4.5;
carWidth = 1.8;

figure('Name','LKA - All Road Scenarios', ...
       'Color','w', ...
       'Position',[100 100 1400 750]);

%% 7.1 ROAD ANIMATION AXES

axRoad = axes('Position',[0.06 0.12 0.66 0.78]);

hold(axRoad,'on');
grid(axRoad,'on');
axis(axRoad,'equal');

% Road boundaries
plot(axRoad,Xleft,Yleft,'k','LineWidth',2);
plot(axRoad,Xright,Yright,'k','LineWidth',2);

% Reference centerline
plot(axRoad,Xroad,Yroad,'k--','LineWidth',1.2);

% Normal vehicle trajectory
trajectory = animatedline(axRoad, ...
    'Color','b','LineWidth',1.8);

% Highlight trajectory during disturbance and recovery
disturbanceTrajectory = animatedline(axRoad, ...
    'Color',[1 0.3 0],'LineWidth',2.8);

% Vehicle body
car = patch(axRoad, ...
    'XData',zeros(1,4), ...
    'YData',zeros(1,4), ...
    'FaceColor',[0.1 0.45 0.9], ...
    'EdgeColor','k');

% Vehicle front indicator
front = plot(axRoad,nan,nan,'r','LineWidth',2);

% Lateral deviation indicator
deviationLine = plot(axRoad,nan,nan,'r--','LineWidth',2);

% Information panel
info = text(axRoad,0.98,0.96,'', ...
    'Units','normalized', ...
    'FontSize',10, ...
    'VerticalAlignment','top', ...
    'HorizontalAlignment','right', ...
    'BackgroundColor','w', ...
    'Margin',6);

xlabel(axRoad,'X position (m)');
ylabel(axRoad,'Y position (m)');

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

title(axRoad,sprintf( ...
    'LKA - Scenario %d | Controller %s | Disturbance %s', ...
    roadScenario,controllerText,disturbanceText));

% Animation limits
margin = 10;
xMargin = 50;

xlim(axRoad,[min([Xroad;Xvehicle])-xMargin ...
             max([Xroad;Xvehicle])+xMargin]);

ylim(axRoad,[min([Yroad;Yvehicle])-margin ...
             max([Yroad;Yvehicle])+margin]);

%% 8. STEERING WHEEL VISUALIZATION

% Separate axes for steering wheel
axWheel = axes('Position',[0.76 0.28 0.20 0.40]);

hold(axWheel,'on');
axis(axWheel,'equal');
axis(axWheel,'off');

xlim(axWheel,[-1.2 1.2]);
ylim(axWheel,[-1.35 1.2]);

% Steering wheel dimensions
wheelRadius = 0.85;

% Outer rim
a = linspace(0,2*pi,150);

plot(axWheel, ...
    wheelRadius*cos(a), ...
    wheelRadius*sin(a), ...
    'Color',[0.12 0.16 0.22], ...
    'LineWidth',9);

% Inner decorative rim
plot(axWheel, ...
    0.72*cos(a), ...
    0.72*sin(a), ...
    'Color',[0.35 0.42 0.52], ...
    'LineWidth',1.5);

% Three rotating spokes
spokeAngles = [90 210 330];

spokes = gobjects(1,3);

for j = 1:3

    angle = deg2rad(spokeAngles(j));

    xEnd = 0.72*cos(angle);
    yEnd = 0.72*sin(angle);

    spokes(j) = plot(axWheel,[0 xEnd],[0 yEnd], ...
        'Color',[0.12 0.16 0.22], ...
        'LineWidth',7);

end

% Central hub
rectangle(axWheel, ...
    'Position',[-0.22 -0.22 0.44 0.44], ...
    'Curvature',[1 1], ...
    'FaceColor',[0.12 0.20 0.32], ...
    'EdgeColor',[0.1 0.55 0.95], ...
    'LineWidth',2);

% Center label
text(axWheel,0,0,'LKA', ...
    'Color','w', ...
    'FontWeight','bold', ...
    'FontSize',10, ...
    'HorizontalAlignment','center');

% Rotating blue indicator
rotationIndicator = plot(axWheel,0,0,'o', ...
    'MarkerSize',9, ...
    'MarkerFaceColor',[0.1 0.65 1], ...
    'MarkerEdgeColor','w');

% Steering angle text
steeringAngleText = text(axWheel,0,-1.08,'0.0 deg', ...
    'HorizontalAlignment','center', ...
    'FontSize',11, ...
    'FontWeight','bold', ...
    'Color',[0.1 0.45 0.9]);

% Illustrative steering ratio
steeringRatio = 12;

title(axWheel,'STEERING WHEEL', ...
    'FontWeight','bold', ...
    'FontSize',11);

%% 9. ANIMATION LOOP

for i = 1:length(t)

    %% 9.1 VEHICLE POSITION AND ORIENTATION

    xc = Xvehicle(i);
    yc = Yvehicle(i);
    theta = psiVehicle(i);

    %% 9.2 UPDATE STEERING WHEEL

    % Convert front-wheel angle into an illustrative
    % steering-wheel rotation using a 12:1 ratio.

    steeringAngle = steeringRatio * delta(i);

    % Rotate all three spokes
    for j = 1:3

        angle = deg2rad(spokeAngles(j)) + steeringAngle;

        xEnd = 0.72*cos(angle);
        yEnd = 0.72*sin(angle);

        set(spokes(j), ...
            'XData',[0 xEnd], ...
            'YData',[0 yEnd]);

    end

    % Update blue indicator
    indicatorAngle = pi/2 + steeringAngle;

set(rotationIndicator, ...
    'XData',wheelRadius*cos(indicatorAngle), ...
    'YData',wheelRadius*sin(indicatorAngle));

    % Display steering wheel angle
    set(steeringAngleText,'String', ...
        sprintf('%.1f deg',rad2deg(steeringAngle)));

    %% 9.3 VEHICLE RECTANGLE

    localX = [-carLength/2 carLength/2 ...
               carLength/2 -carLength/2];

    localY = [-carWidth/2 -carWidth/2 ...
               carWidth/2 carWidth/2];

    rotatedX = xc + localX*cos(theta) - localY*sin(theta);
    rotatedY = yc + localX*sin(theta) + localY*cos(theta);

    %% 9.4 VEHICLE COLOR AND DISTURBANCE HIGHLIGHT

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

    % Lateral deviation
    set(deviationLine, ...
        'XData',[Xroad(i),Xvehicle(i)], ...
        'YData',[Yroad(i),Yvehicle(i)]);

    %% 9.5 DISTURBANCE STATUS

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

    %% 9.6 INFORMATION PANEL

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
             t(i),controllerText,ey(i),ephi(i),delta(i)));

    end

    %% 9.7 REFRESH ANIMATION

    drawnow;

    %% 9.8 ANIMATION SPEED

    if disturbanceEnabled == 1 && highlightActive(i)

        pause(0.15);  % Slow motion during disturbance and recovery

    else

        pause(0.03);  % Normal animation speed

    end

end

disp('LKA animation completed.');
