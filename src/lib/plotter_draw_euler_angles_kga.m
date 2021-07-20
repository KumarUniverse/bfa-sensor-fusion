
function plotter_draw_euler_angles_kga(data, HIDE_ROLL)
    % Plot the Euler angles (yaw, pitch, roll) using MATLAB's
    % implementation of the KGA algorithm to calculate orientation.
    addpath('src/quaternion_library');  % include quaternion library
    addpath('data');

    Fs = 96; % frequency of the data.

    data = table2array(data);
    time = data(:, 1);
    accel = data(:, 2:4);
    gyro = data(:, 5:7);
    mag = data(:, 8:10);

    fuse = complementaryFilter('SampleRate', Fs);
    q = fuse(accel, gyro, mag);

    writematrix(compact(q), 'out/quat_matlab.csv');
    
    euler_matrix = eulerd(q, 'ZYX', 'frame');
    figure('Name', 'Euler Angles (KGA)');
    hold on;
    plot(time, euler_matrix(:,1), 'r'); % phi or yaw
    plot(time, euler_matrix(:,2), 'g'); % theta or pitch
    if not(HIDE_ROLL) % if true, don't graph roll
        plot(time, euler_matrix(:,3), 'b'); % psi or roll
    end
    title('Euler Angles (KGA)');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    if HIDE_ROLL % if true, don't graph roll
        legend('\phi', '\theta');
    else
        legend('\phi', '\theta', '\psi');
    end
    hold off;
end


