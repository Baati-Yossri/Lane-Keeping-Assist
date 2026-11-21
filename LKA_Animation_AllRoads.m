
%% LKA ANIMATION - ALL ROAD SCENARIOS

clc;

%% 1. Extract simulation results

t = out.ey_out.Time(:);
ey = out.ey_out.Data(:);
ephi = out.ephi_out.Data(:);
delta = out.delta_out.Data(:);
u = out.u_out.Data(:);

%% 2. Obtain curvature at simulation timestamps

kappa = interp1(kappaSignal.Time, ...
                kappaSignal.Data, t, 'linear', 'extrap');

%% 3. Reconstruct the reference road

% Road heading
psiRoad = cumtrapz(t, v .* kappa);

% Reference road coordinates
Xroad = cumtrapz(t, v .* cos(psiRoad));
Yroad = cumtrapz(t, v .* sin(psiRoad));



%% 4. Calculate vehicle trajectory

psiVehicle = psiRoad + ephi;

% Initial vehicle position
X0 = Xroad(1) - ey(1)*sin(psiRoad(1));
Y0 = Yroad(1) + ey(1)*cos(psiRoad(1));

if controllerEnabled == 1

    % Controller ON: follow the road using lateral error
    Xvehicle = Xroad - ey.*sin(psiRoad);
    Yvehicle = Yroad + ey.*cos(psiRoad);

else

    % Controller OFF: maintain initial heading
    initialHeading = psiVehicle(1);

    Xvehicle = X0 + v*t*cos(initialHeading);
    Yvehicle = Y0 + v*t*sin(initialHeading);

end

%% 5. Calculate road boundaries

roadWidth = 3.6;
halfWidth = roadWidth / 2;

% Left boundary
Xleft = Xroad - halfWidth .* sin(psiRoad);
Yleft = Yroad + halfWidth .* cos(psiRoad);

% Right boundary
Xright = Xroad + halfWidth .* sin(psiRoad);
Yright = Yroad - halfWidth .* cos(psiRoad);

disp('Road and vehicle trajectories reconstructed.');






%% 6. ANIMATION SETTINGS

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
plot(Xroad,Yroad,'k--','LineWidth',1.2);

% Vehicle trajectory
trajectory = animatedline('Color','b','LineWidth',1.5);

% Vehicle body
car = patch('XData',zeros(1,4), ...
            'YData',zeros(1,4), ...
            'FaceColor',[0.1 0.45 0.9], ...
            'EdgeColor','k');

% Vehicle front
front = plot(nan,nan,'r','LineWidth',2);

% Information panel
info = text(0.98,0.96,'', ...
    'Units','normalized', ...
    'FontSize',11, ...
    'VerticalAlignment','top', ...
    'HorizontalAlignment','right', ...
    'BackgroundColor','w');

xlabel('X position (m)');
ylabel('Y position (m)');
title(sprintf('LKA - Road Scenario %d',roadScenario));

% Animation limits
margin = 10;
xlim([min(Xroad)-margin max(Xroad)+margin]);
ylim([min(Yroad)-margin max(Yroad)+margin]);

%% 7. ANIMATION LOOP

for i = 1:length(t)

    % Vehicle center
    xc = Xvehicle(i);
    yc = Yvehicle(i);
    theta = psiVehicle(i);

    % Vehicle rectangle corners
    localX = [-carLength/2 carLength/2 ...
               carLength/2 -carLength/2];

    localY = [-carWidth/2 -carWidth/2 ...
               carWidth/2 carWidth/2];

    % Rotate vehicle
    rotatedX = xc + localX*cos(theta) - localY*sin(theta);
    rotatedY = yc + localX*sin(theta) + localY*cos(theta);

    % Update vehicle
    set(car,'XData',rotatedX,'YData',rotatedY);

    % Front indicator
    set(front, ...
        'XData',[xc xc+(carLength/2)*cos(theta)], ...
        'YData',[yc yc+(carLength/2)*sin(theta)]);

    % Update trajectory
    addpoints(trajectory,xc,yc);

    % Update information
    set(info,'String',sprintf( ...
        'Time: %.2f s\nLateral error: %.3f m\nHeading error: %.4f rad\nSteering: %.3f rad', ...
        t(i),ey(i),ephi(i),delta(i)));

    drawnow;
    pause(0.02);

end

disp('LKA animation completed.');
