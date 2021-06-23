
function plotter_draw_euler_angles_xio(TEST_NAME)
    % Plot the Euler angles (yaw, pitch, roll) using the
    % X-IO toolbox to calculate orientation.
    % TEST_NAME is the name of the data file.
    addpath('src/quaternion_library');  % include quaternion library
    addpath('data');
    
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


