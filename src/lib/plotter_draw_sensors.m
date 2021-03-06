% Note that the Euler angle plot shows erratic behaviour in phi and psi
% when theta approaches �90 degrees. This due to a singularity in the Euler
% angle sequence known as 'Gimbal lock'.  This issue does not exist for a
% quaternion or rotation matrix representation.

function plotter_draw_sensors(data)
    % Reads the sensor data from a file and plots the
    % graphs of the sensor readings for each sensor.
    addpath('src/quaternion_library');  % include quaternion library
    addpath('data');

    % Categorize the data into their respective sensor data.
    data = table2array(data);
    Accelerometer = data(:,2:4);
    Gyroscope = data(:,5:7);
    Magnetometer = data(:,8:10);
    time = data(:,1);

    figure('Name', 'Sensor Data');

    % Plot the Gyroscope data into its own separate subplot.
    axis(1) = subplot(3,1,1);
    hold on;
    plot(time, Gyroscope(:,1), 'r');
    plot(time, Gyroscope(:,2), 'g');
    plot(time, Gyroscope(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Angular rate (deg/s)');
    title('Gyroscope');
    hold off;

    % Plot the Accelerometer data into its own separate subplot.
    axis(2) = subplot(3,1,2);
    hold on;
    plot(time, Accelerometer(:,1), 'r');
    plot(time, Accelerometer(:,2), 'g');
    plot(time, Accelerometer(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    title('Accelerometer');
    hold off;

    % Plot the magnetometer data into its own separate subplot.
    axis(3) = subplot(3,1,3);
    hold on;
    plot(time, Magnetometer(:,1), 'r');
    plot(time, Magnetometer(:,2), 'g');
    plot(time, Magnetometer(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Flux (G)');
    title('Magnetometer');
    hold off;
    linkaxes(axis, 'x');
end
