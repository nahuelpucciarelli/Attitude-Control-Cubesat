%% CUBESAT 3D ANIMATION SCRIPT WITH REAL-TIME PLOTS (OPTIMIZED)
% Run the simulation is complete.

clc; close all;
set(0, 'DefaultFigureWindowStyle', 'normal');
set(0, 'DefaultFigureVisible', 'on');

%% Extract and Convert Data
time     = out.angles.Time;
eul_rad  = out.angles.Data;      % [Yaw, Pitch, Roll] in Radians (ZYX)
omega_rw = out.omega_rw.Data;    % Reaction wheel speeds [wx, wy, wz] in rad/s

roll_deg  = rad2deg(eul_rad(:,3));
pitch_deg = rad2deg(eul_rad(:,2));
yaw_deg   = rad2deg(eul_rad(:,1));
omega_rw_rpm = omega_rw * (60/(2*pi));

%% Color Scheme (X=Red, Y=Green, Z=Cyan)
colors = struct('x', 'r', 'y', 'g', 'z', 'c', ...
                'xrgb', [1 0 0], 'yrgb', [0 1 0], 'zrgb', [0 1 1]);

%% Animation Parameters
indices      = 1:speed_factor:length(time);
target_fps   = 30;
frame_delay  = 1 / target_fps;

% GIF Export
gif_filename = 'cubesat_animation.gif';
gif_delay    = 1 / 30;

%% Setup Figure
figWidth  = 0.8;   % 80% of screen width
figHeight = 0.8;   % 80% of screen height

fig = figure('Color', 'k', 'Name', 'CubeSat Attitude Animation', ...
             'NumberTitle', 'off', 'Units', 'normalized', ...
             'Position', [(1-figWidth)/4 (1-figHeight)/3 figWidth*1.1 figHeight]);

%% 3D Animation Axes (Left)
ax_3d = subplot(1, 2, 1);
set(ax_3d, 'Position', [0.02 0.05 0.48 0.9], 'Color', 'k', ...
    'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
axis equal; grid on; view(142, 30);
xlim([-0.2 0.2]); ylim([-0.2 0.2]); zlim([-0.2 0.2]);
xlabel('X'); ylabel('Y'); zlabel('Z'); hold on;

light('Position', [100 30 50], 'Style', 'infinite', 'Parent', ax_3d);
lighting gouraud;

%% CubeSat Body
L = 0.1; H = 0.1135;
vertices = [-L/2 -L/2 -H/2; L/2 -L/2 -H/2; L/2 L/2 -H/2; -L/2 L/2 -H/2;
            -L/2 -L/2  H/2; L/2 -L/2  H/2; L/2 L/2  H/2; -L/2 L/2  H/2];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

cube_colors = repmat([0.8401 0.6822 0.4472], 6, 1);
cube_colors(2,:) = [0.05 0.15 0.35];  % Solar panel

cube = patch('Vertices', vertices, 'Faces', faces, 'FaceVertexCData', cube_colors, ...
    'FaceColor', 'flat', 'EdgeColor', [0.44 0.44 0.45], 'FaceAlpha', alpha, ...
    'LineWidth', 2, 'Parent', ax_3d);
material(cube, 'shiny'); 
set(cube, 'SpecularStrength', 0.8, 'DiffuseStrength', 0.9, 'AmbientStrength', 0.5);

t_group = hgtransform('Parent', ax_3d);
set(cube, 'Parent', t_group);

% Solar panel grid
panel_size = L-0.01; grid_div = 3;
for i = 0:grid_div
    y_coord = -panel_size/2 + i*(panel_size/grid_div);
    x_coord = -panel_size/2 + i*(panel_size/grid_div);
    line([-panel_size/2, panel_size/2], [y_coord, y_coord], [H/2, H/2], ...
        'Color', [0.86 0.82 0.82], 'LineWidth', 1, 'Parent', t_group);
    line([x_coord, x_coord], [-panel_size/2, panel_size/2], [H/2, H/2], ...
        'Color', [0.86 0.82 0.82], 'LineWidth', 1, 'Parent', t_group);
end

% Body Axes
L_axis = 0.15;
line([0 L_axis], [0 0], [0 0], 'Color', colors.x, 'LineWidth', 1.5, 'Parent', t_group);
line([0 0], [0 L_axis], [0 0], 'Color', colors.y, 'LineWidth', 1.5, 'Parent', t_group);
line([0 0], [0 0], [0 L_axis], 'Color', colors.z, 'LineWidth', 1.5, 'Parent', t_group);
text(L_axis, 0, 0, 'X', 'Color', colors.x, 'FontSize', 15, 'Parent', t_group);
text(0, L_axis, 0, 'Y', 'Color', colors.y, 'FontSize', 15, 'Parent', t_group);
text(0, 0, L_axis, 'Z', 'Color', colors.z, 'FontSize', 15, 'Parent', t_group);

%% Camera
t_camera = hgtransform('Parent', t_group);
camera_size = 0.06; camera_x_pos = 0.055;
sx = camera_size * 0.1; sy = camera_size / 2; sz = camera_size / 2;
cam_vertices = [-sx -sy -sz; sx -sy -sz; sx sy -sz; -sx sy -sz;
                -sx -sy  sz; sx -sy  sz; sx sy  sz; -sx sy  sz];
cam_vertices(:,1) = cam_vertices(:,1) + camera_x_pos;
cam_faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
cam_colors = repmat([0.2 0.2 0.2], 6, 1);

camera_cube = patch('Vertices', cam_vertices, 'Faces', cam_faces, ...
    'FaceVertexCData', cam_colors, 'FaceColor', 'flat', ...
    'EdgeColor', [0.44 0.44 0.45], 'FaceAlpha', alpha, 'LineWidth', 2, 'Parent', t_camera);
material(camera_cube, 'metal'); 
set(camera_cube, 'SpecularStrength', 0.8, 'DiffuseStrength', 0.9, 'AmbientStrength', 0.5);

% Lens
lens_radius = min([sy sz]) * 0.8;
theta_cam = linspace(0, 2*pi, 30);
lens_y = lens_radius * cos(theta_cam);
lens_z = lens_radius * sin(theta_cam);
lens_x = ones(size(theta_cam)) * (camera_x_pos + sx + 0.001);
fill3(lens_x, lens_y, lens_z, [0.2 0.2 0.4], 'EdgeColor', 'none', 'Parent', t_camera);

%% Reaction Wheels
rw_rad   = rw_radius;
rw_thick = rw_thickness;
wheel_pos = {pos_x_input, pos_y_input, pos_z_input};
wheel_colors = {colors.xrgb, colors.yrgb, colors.zrgb};
wheel_names = {'x', 'y', 'z'};

% Pre-compute cylinder geometry
[cyl_y, cyl_z, cyl_x] = cylinder(rw_rad, 100);
cyl_x = cyl_x * rw_thick - rw_thick / 2;
theta = linspace(0, 2*pi, 30);
cap_y = rw_rad * cos(theta);
cap_z = rw_rad * sin(theta);
mark_theta = linspace(-0.01*pi, 0.01*pi, 20);
mark_y = rw_rad * cos(mark_theta);
mark_z = rw_rad * sin(mark_theta);

% Create wheels
t_wheels = cell(3,1);
for w = 1:3
    t_wheels{w} = hgtransform('Parent', t_group);
    color = wheel_colors{w};
    
    if w == 1  % X-wheel
        surf(cyl_x, cyl_y, cyl_z, 'FaceColor', color, 'EdgeColor', 'none', 'Parent', t_wheels{w});
        fill3(ones(size(theta))*( rw_thick/2), cap_y, cap_z, color, 'EdgeColor', 'none', 'Parent', t_wheels{w});
        fill3(ones(size(theta))*(-rw_thick/2), cap_y, cap_z, color, 'EdgeColor', 'none', 'Parent', t_wheels{w});
        for j = 1:length(mark_theta)
            line([-rw_thick/2, rw_thick/2], [mark_y(j), mark_y(j)], [mark_z(j), mark_z(j)], ...
                'Color', 'w', 'LineWidth', 2, 'Parent', t_wheels{w});
        end
    elseif w == 2  % Y-wheel
        surf(cyl_y, cyl_x, cyl_z, 'FaceColor', color, 'EdgeColor', 'none', 'Parent', t_wheels{w});
        fill3(cap_y, ones(size(theta))*( rw_thick/2), cap_z, color, 'EdgeColor', 'none', 'Parent', t_wheels{w});
        fill3(cap_y, ones(size(theta))*(-rw_thick/2), cap_z, color, 'EdgeColor', 'none', 'Parent', t_wheels{w});
        for j = 1:length(mark_theta)
            line([mark_y(j), mark_y(j)], [-rw_thick/2, rw_thick/2], [mark_z(j), mark_z(j)], ...
                'Color', 'w', 'LineWidth', 2, 'Parent', t_wheels{w});
        end
    else  % Z-wheel
        surf(cyl_y, cyl_z, cyl_x, 'FaceColor', color, 'EdgeColor', 'none', 'Parent', t_wheels{w});
        fill3(cap_y, cap_z, ones(size(theta))*( rw_thick/2), color, 'EdgeColor', 'none', 'Parent', t_wheels{w});
        fill3(cap_y, cap_z, ones(size(theta))*(-rw_thick/2), color, 'EdgeColor', 'none', 'Parent', t_wheels{w});
        for j = 1:length(mark_theta)
            line([mark_y(j), mark_y(j)], [mark_z(j), mark_z(j)], [-rw_thick/2, rw_thick/2], ...
                'Color', 'w', 'LineWidth', 2, 'Parent', t_wheels{w});
        end
    end
end

%% Real-Time Plots
% Euler Angles
ax_angles = subplot(2, 2, 2);
set(ax_angles, 'Position', [0.55 0.55 0.42 0.38], 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold on; grid on;
title('Satellite Attitude (Euler Angles)', 'Color', 'w', 'FontSize', 11);
ylabel('Angle (deg)', 'Color', 'w');
line_roll  = plot(NaN, NaN, colors.x, 'LineWidth', 2);
line_pitch = plot(NaN, NaN, colors.y, 'LineWidth', 2);
line_yaw   = plot(NaN, NaN, colors.z, 'LineWidth', 2);
ylim([-200 200]); xlim([0 time(end)]);
legend({'Roll (X)', 'Pitch (Y)', 'Yaw (Z)'}, 'TextColor', 'w', 'Location', 'northeast', 'Color', [0 0 0]);

% Wheel Speed
ax_wheels = subplot(2, 2, 4);
set(ax_wheels, 'Position', [0.55 0.08 0.42 0.38], 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold on; grid on;
title('Reaction Wheel Speeds', 'Color', 'w', 'FontSize', 11);
ylabel('Speed (RPM)', 'Color', 'w'); xlabel('Time (s)', 'Color', 'w');
line_wx = plot(NaN, NaN, colors.x, 'LineWidth', 2);
line_wy = plot(NaN, NaN, colors.y, 'LineWidth', 2);
line_wz = plot(NaN, NaN, colors.z, 'LineWidth', 2);
yline(5600, 'w--', 'Max RPM', 'LineWidth', 1, 'LabelHorizontalAlignment', 'left');
yline(-5600, 'w--', 'Min RPM', 'LineWidth', 1, 'LabelHorizontalAlignment', 'left');
ylim([-6500 6500]); xlim([0 time(end)]);
legend({'RW X', 'RW Y', 'RW Z'}, 'TextColor', 'w', 'Location', 'northeast', 'Color', [0 0 0]);

%% Pre-compute Transformations with proper wheel angle integration
fprintf('Pre-computing transformations...\n');
num_frames = length(indices);
R_body = cell(num_frames, 1);
R_wheels = cell(num_frames, 3);

% Integrate wheel angles
wheel_angles_full = zeros(length(time), 3);
for i = 2:length(time)
    dt = time(i) - time(i-1);
    wheel_angles_full(i, :) = wheel_angles_full(i-1, :) + omega_rw(i, :) * dt;
end

% Sample at animation indices
wheel_angles = wheel_angles_full(indices, :);

for idx = 1:num_frames
    i = indices(idx);
    
    % Body rotation (ZYX Euler)
    Rz = makehgtform('zrotate', eul_rad(i, 1));
    Ry = makehgtform('yrotate', eul_rad(i, 2));
    Rx = makehgtform('xrotate', eul_rad(i, 3));
    R_body{idx} = Rz * Ry * Rx;
    
    % Wheel transforms with integrated angles
    R_wheels{idx, 1} = makehgtform('translate', wheel_pos{1}) * makehgtform('xrotate', wheel_angles(idx, 1));
    R_wheels{idx, 2} = makehgtform('translate', wheel_pos{2}) * makehgtform('yrotate', wheel_angles(idx, 2));
    R_wheels{idx, 3} = makehgtform('translate', wheel_pos{3}) * makehgtform('zrotate', wheel_angles(idx, 3));
end

%% Animation Loop
title_handle = title(ax_3d, '', 'Color', 'w', 'FontSize', 12, 'FontName', 'FixedWidth');
time_buffer = []; roll_buffer = []; pitch_buffer = []; yaw_buffer = [];
wx_buffer = []; wy_buffer = []; wz_buffer = [];

fprintf('Starting animation...\n');
fprintf('Total frames: %d\n', num_frames);

for idx = 1:num_frames
    i = indices(idx);
    
    % Apply transformations
    set(t_group, 'Matrix', R_body{idx});
    for w = 1:3
        set(t_wheels{w}, 'Matrix', R_wheels{idx, w});
    end
    
    % Update title
    set(title_handle, 'String', sprintf('T:%6.0fs | R:%5.0f° P:%5.0f° Y:%5.0f° | RW:[%5.0f,%5.0f,%5.0f]RPM', ...
        time(i), roll_deg(i), pitch_deg(i), yaw_deg(i), ...
        omega_rw_rpm(i,1), omega_rw_rpm(i,2), omega_rw_rpm(i,3)));
    
    % Update plot data
    time_buffer = [time_buffer; time(i)];
    roll_buffer = [roll_buffer; roll_deg(i)];
    pitch_buffer = [pitch_buffer; pitch_deg(i)];
    yaw_buffer = [yaw_buffer; yaw_deg(i)];
    wx_buffer = [wx_buffer; omega_rw_rpm(i,1)];
    wy_buffer = [wy_buffer; omega_rw_rpm(i,2)];
    wz_buffer = [wz_buffer; omega_rw_rpm(i,3)];
    
    set(line_roll,  'XData', time_buffer, 'YData', roll_buffer);
    set(line_pitch, 'XData', time_buffer, 'YData', pitch_buffer);
    set(line_yaw,   'XData', time_buffer, 'YData', yaw_buffer);
    set(line_wx, 'XData', time_buffer, 'YData', wx_buffer);
    set(line_wy, 'XData', time_buffer, 'YData', wy_buffer);
    set(line_wz, 'XData', time_buffer, 'YData', wz_buffer);
    
    drawnow limitrate;
    
    % GIF Export
    if save_gif
        frame = getframe(fig);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if idx == 1
            imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', gif_delay);
        else
            imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', gif_delay);
        end
    end
    
    pause(frame_delay);
    if ~isvalid(fig), break; end
end

fprintf('Animation finished\n');
if save_gif, fprintf('GIF saved successfully\n'); end