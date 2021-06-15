% Note that the Euler angle plot shows erratic behaviour in phi and psi
% when theta approaches �90 degrees. This due to a singularity in the Euler
% angle sequence known as 'Gimbal lock'.  This issue does not exist for a
% quaternion or rotation matrix representation.

addpath('../quaternion_library');  % include quaternion library
% Import and plot sensor data
% addpath("../Sensor Fusion/data");
% addpath("../Data Sets");
addpath("../data");

function draw_sensors(TEST_NAME)
    % Reads the sensor data from a file and plots the
    % graphs of the sensor readings for each sensor.
    % TEST_NAME is the name of the data file.
    % Eg: TEST_NAME = "euler_angles_2_calibrated";
    data = readmatrix(TEST_NAME);
    data = data(1:10:end,:); % downsampling.

    % Categorize the data into their respective sensor data.
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

function draw_euler_angles_xio(TEST_NAME)
    % Plot the Euler angles (yaw, pitch, roll) using the
    % X-IO toolbox to calculate orientation.
    % TEST_NAME is the name of the data file.
    data = readmatrix(TEST_NAME);
    data = data(1:10:end,:);

    Accelerometer = data(:,2:4);
    Gyroscope = data(:,5:7);
    Magnetometer = data(:,8:10);
    time = data(:,1);
    
    % ["Madgwick", "Mahony", "MagMahony"]
    FILTER_TYPE = "Madgwick";
    
    % Process sensor data through algorithm
    switch FILTER_TYPE
        case "Madgwick"
            AHRS = MadgwickAHRS('SamplePeriod', 1/96, 'Beta', 0.9);
        case "Mahony"
            AHRS = MahonyAHRS('SamplePeriod', 1/96, 'Beta', 0.1);
        case "MagMahony"
            AHRS = MagMahonyAHRS('SamplePeriod', 1/96, 'Kp', 0.5);
    end

    quaternion = zeros(length(time), 4);
    for t = 1:length(time)
        AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
        quaternion(t, :) = AHRS.Quaternion;
    end

    writematrix(quaternion, "out/quat_xio.csv")
    
    % Plot algorithm output as Euler angles
    % The first and third Euler angles in the sequence (phi and psi) become
    % unreliable when the middle angles of the sequence (theta) approaches �90
    % degrees. This problem commonly referred to as Gimbal Lock.
    % See: http://en.wikipedia.org/wiki/Gimbal_lock

    euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

    figure('Name', 'Euler Angles');
    hold on;
    plot(time, euler(:,1), 'r'); % phi or yaw
    plot(time, euler(:,2), 'g'); % theta or pitch
    plot(time, euler(:,3), 'b'); % psi or roll
    title('Euler angles');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    legend('\phi', '\theta', '\psi');
    hold off;
end

function draw_euler_angles_kga(TEST_NAME)
    % Plot the Euler angles (yaw, pitch, roll) using MATLAB's
    % implementation of the KGA algorithm to calculate orientation.
    % TEST_NAME is the name of the data file.
    Fs = 96; % frequency of the data.

    data = readmatrix(TEST_NAME);
    data = data(1:10:end,:);

    accel = data(:, 2:4);
    gyro = deg2rad(data(:, 5:7));
    mag = data(:, 8:10);

    fuse = complementaryFilter("SampleRate", Fs);
    q = fuse(accel, gyro, mag);

    writematrix(compact(q), "out/quat_matlab.csv");

    plot(eulerd(q, 'ZYX', 'frame'));
    title('Orientation Estimate');
    legend('Z-rotation', 'Y-rotation', 'X-rotation');
    ylabel('Degrees');
end
