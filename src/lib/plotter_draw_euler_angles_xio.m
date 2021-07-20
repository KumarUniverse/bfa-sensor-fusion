
function plotter_draw_euler_angles_xio(data, HIDE_ROLL)
    % Plot the Euler angles (yaw, pitch, roll) using the
    % X-IO toolbox to calculate orientation.
    addpath('src/quaternion_library');  % include quaternion library
    addpath('src/AHRS');

    data = table2array(data);
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

    quaternion = zeros(height(time), 4);
    for t = 1:height(time)
        AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
        quaternion(t, :) = AHRS.Quaternion;
    end

    writematrix(quaternion, 'out/quat_xio.csv');

    % Use conjugate for sensor frame relative to Earth and convert to degrees.
    euler_matrix = quatern2euler(quaternConj(quaternion)) * (180/pi);

    figure('Name', 'Euler Angles (XIO)');
    hold on;
    plot(time, euler_matrix(:,3), 'r'); % phi or yaw
    plot(time, euler_matrix(:,2), 'g'); % theta or pitch
    if not(HIDE_ROLL) % if true, don't graph roll
        plot(time, euler_matrix(:,1), 'b'); % psi or roll
    end
    title('Euler angles (XIO)');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    if HIDE_ROLL % if true, don't graph roll
        legend('\phi', '\theta');
    else
        legend('\phi', '\theta', '\psi');
    end
    hold off;
end


