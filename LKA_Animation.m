
%% ============================================================
%  LANE KEEPING ASSIST - CURVED ROAD VEHICLE ANIMATION
%  Based on actual MATLAB/Simulink simulation outputs
% =============================================================

clearvars -except out
close all;
clc;

%% 1. VEHICLE AND ROAD PARAMETERS

v = 30;                 % Vehicle speed (m/s)
kappa_r = 0.005;        % Road curvature (1/m)

lane_width = 3.6;       % Lane width (m)

car_length = 4.5;       % Vehicle length (m)
car_width = 1.8;        % Vehicle width (m)

animation_delay = 0.04;

%% 2. EXTRACT SIMULINK OUTPUTS

t = out.ey_out.Time(:);

ey = out.ey_out.Data(:);
ephi = out.ephi_out.Data(:);
delta = out.delta_out.Data(:);
u = out.u_out.Data(:);

% Verify signal lengths
assert(length(t) == length(ey), 'ey signal size mismatch');
assert(length(t) == length(ephi), 'ephi signal size mismatch');
assert(length(t) == length(delta), 'delta signal size mismatch');
assert(length(t) == length(u), 'u signal size mismatch');

%% 3. RECONSTRUCT THE CURVED ROAD

% Road tangent angle
psi_road = v * kappa_r * t;

% Reference road centerline
X_road = sin(psi_road) / kappa_r;
Y_road = (1 - cos(psi_road)) / kappa_r;

% Normal direction to the road
normal_x = -sin(psi_road);
normal_y = cos(psi_road);

% Vehicle position relative to road centerline
X_vehicle = X_road + ey .* normal_x;
Y_vehicle = Y_road + ey .* normal_y;

% Vehicle orientation
theta = psi_road + ephi;

%% 4. CONSTRUCT ROAD BOUNDARIES

half_lane = lane_width / 2;

% Left boundary
X_left = X_road + half_lane * normal_x;
Y_left = Y_road + half_lane * normal_y;

% Right boundary
X_right = X_road - half_lane * normal_x;
Y_right = Y_road - half_lane * normal_y;

%% 5. CREATE THE VEHICLE SHAPE

% Rectangle centered at vehicle origin
car_local = [
    -car_length/2, -car_width/2;
     car_length/2, -car_width/2;
     car_length/2,  car_width/2;
    -car_length/2,  car_width/2
];

% Front indicator line
front_local = [
    car_length/2, -car_width/2;
    car_length/2,  car_width/2
];

%% 6. CREATE FIGURE

figure('Name','LKA - Curved Road Simulation', ...
       'Color','w','Position',[100 100 1100 700]);

hold on;
grid on;
axis equal;

% Road boundaries
plot(X_left,Y_left,'k-','LineWidth',2, ...
    'DisplayName','Road boundaries');

plot(X_right,Y_right,'k-','LineWidth',2, ...
    'HandleVisibility','off');

% Road centerline
plot(X_road,Y_road,'k--','LineWidth',1.2, ...
    'DisplayName','Reference trajectory');

% Vehicle trajectory
trajectory = plot(NaN,NaN,'b-', ...
    'LineWidth',1.5,'DisplayName','Vehicle trajectory');

% Initial vehicle position
R = [cos(theta(1)), -sin(theta(1));
     sin(theta(1)),  cos(theta(1))];

car_world = (R * car_local')';

car_world(:,1) = car_world(:,1) + X_vehicle(1);
car_world(:,2) = car_world(:,2) + Y_vehicle(1);

car = patch(car_world(:,1),car_world(:,2), ...
    [0.1 0.4 0.9], ...
    'EdgeColor','k','LineWidth',1.5, ...
    'DisplayName','Vehicle');

% Vehicle front indicator
front_world = (R * front_local')';

front_world(:,1) = front_world(:,1) + X_vehicle(1);
front_world(:,2) = front_world(:,2) + Y_vehicle(1);

front_line = plot(front_world(:,1),front_world(:,2), ...
    'r-','LineWidth',3,'HandleVisibility','off');

% Current vehicle position marker
position_marker = plot(X_vehicle(1),Y_vehicle(1), ...
    'ro','MarkerFaceColor','r','MarkerSize',5, ...
    'HandleVisibility','off');

% Information panel
info = text(0.02,0.97,'','Units','normalized', ...
    'VerticalAlignment','top', ...
    'FontSize',11,'FontWeight','bold', ...
    'BackgroundColor','w','Margin',6);

% Camera limits
margin = 15;

xlim([min(X_road)-margin, max(X_road)+margin]);
ylim([min(Y_road)-margin, max(Y_road)+margin]);

xlabel('Longitudinal position X (m)');
ylabel('Lateral position Y (m)');

title('Lane Keeping Assist - Curved Road Simulation');

legend('Location','best');

%% 7. ANIMATION LOOP

for k = 1:length(t)

    % Rotation matrix
    R = [cos(theta(k)), -sin(theta(k));
         sin(theta(k)),  cos(theta(k))];

    % Rotate vehicle body
    car_world = (R * car_local')';

    % Translate vehicle
    car_world(:,1) = car_world(:,1) + X_vehicle(k);
    car_world(:,2) = car_world(:,2) + Y_vehicle(k);

    % Update vehicle
    set(car,'XData',car_world(:,1), ...
            'YData',car_world(:,2));

    % Update front indicator
    front_world = (R * front_local')';

    front_world(:,1) = front_world(:,1) + X_vehicle(k);
    front_world(:,2) = front_world(:,2) + Y_vehicle(k);

    set(front_line,'XData',front_world(:,1), ...
                   'YData',front_world(:,2));

    % Update trajectory
    set(trajectory,'XData',X_vehicle(1:k), ...
                   'YData',Y_vehicle(1:k));

    % Update vehicle marker
    set(position_marker,'XData',X_vehicle(k), ...
                        'YData',Y_vehicle(k));

    % Update information panel
    set(info,'String',sprintf([ ...
        'Time: %.2f s\n' ...
        'Speed: %.1f m/s\n' ...
        'Lateral error: %.4f m\n' ...
        'Heading error: %.6f rad\n' ...
        'Steering angle: %.4f rad\n' ...
        'Controller command: %.4f'], ...
        t(k),v,ey(k),ephi(k),delta(k),u(k)));

    title(sprintf('LKA Curved Road Simulation - %.2f s',t(k)));

    drawnow;
    pause(animation_delay);

end

%% 8. DISPLAY FINAL RESULTS

fprintf('\n===== CURVED ROAD ANIMATION RESULTS =====\n');

fprintf('Road curvature: %.4f 1/m\n',kappa_r);
fprintf('Road radius: %.2f m\n',1/kappa_r);

fprintf('Initial lateral error: %.4f m\n',ey(1));
fprintf('Final lateral error: %.4f m\n',ey(end));

fprintf('Initial heading error: %.6f rad\n',ephi(1));
fprintf('Final heading error: %.8f rad\n',ephi(end));

fprintf('Maximum steering angle: %.6f rad\n', ...
    max(abs(delta)));

fprintf('=========================================\n');
