
function plotter_draw_euler_angles_kga(data)
    % Plot the Euler angles (yaw, pitch, roll) using MATLAB's
    % implementation of the KGA algorithm to calculate orientation.
    % TEST_NAME is the name of the data file.
    addpath('src/quaternion_library');  % include quaternion library
    addpath('data');

    Fs = 96; % frequency of the data.

    %data = readmatrix(TEST_NAME);
    %data = data(1:10:end,:);

    accel = table2array(data(:, 2:4));
    gyro = table2array(data(:, 5:7));
    mag = table2array(data(:, 8:10));

    fuse = complementaryFilter('SampleRate', Fs);
    q = fuse(accel, gyro, mag);

    writematrix(compact(q), 'out/quat_matlab.csv');

    plot(eulerd(q, 'ZYX', 'frame'));
    title('Orientation Estimate');
    legend('Z-rotation', 'Y-rotation', 'X-rotation');
    ylabel('Degrees');
end


