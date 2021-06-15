% Keeping a Good Attitude (KGA) Algorithm
import lib.constants.*
import lib.csv_reader
import lib.mag_calibration
import lib.plotter
addpath('quaternion_library');  % include quaternion library

%======================================

TEST_NAME = 'euler_angles_2';

% booleans:
% Note: Only those booleans that were found to be necessary in
% kga_quat.py are used in this script.
CALIBRATE_MAG = true;
USE_PRECALC_MAG = false;
CORRECT_MAG_AXES = true;
NORM_HEADING = true;
USE_ADAPTIVE_GAIN = true;
UPDATE_GYRO_BIAS = true;

% constants:
GAIN = 0.01;
BIAS_ALPHA = 0.01;
GYRO_THRESHOLD = 0.2;
ACC_THRESHOLD = 0.1;
DELTA_GYRO_THRESHOLD = 0.1;

%======================================

M = [0.56144721 -0.01910871 -0.01292889; ...
     -0.01910871  0.6276801  -0.00568568; ...
     -0.01292889 -0.00568568  0.53873008];
     
n = [-13.60233683; -1.60856291; 11.10481335];
    
d = -390.59292573690266;

%======================================

disp('KGA algorithm started.');
disp(['Reading test ', TEST_NAME, '...']);

% read test data at 96 samples/second and convert gyro data to rads
[data, params] = csv_reader.read(TEST_NAME, 'same_sps', true, ...
    'correct_axes', CORRECT_MAG_AXES, 'convert_to_rads', true, ...
    'apply_gyro_bias', true);

% normalized cutoff frequency = cutpff frequency / (2 * sample rate)
ORDER = 10;
CUTOFF_FREQ = 50;
SAMPLE_RATE = 960;
NORM_CUTOFF_FREQ = CUTOFF_FREQ / (2 * SAMPLE_RATE);

% Note: Smoothing is not applied to the acc data.

disp('Test read');

% TODO: is this needed?
if CALIBRATE_MAG
    disp('Calibrating magnetometer data...');
    
    if USE_PRECALC_MAG
        data(MAG_COLS) = mag_calibration.calibrate(data(MAG_COLS), M, n, d);
    else
        data(MAG_COLS) = mag_calibration.calibrate(data(MAG_COLS));
    end
    
    disp('Magnetometer calibration complete.');
end

% DEBUG code not included

disp('Calculating initial orientation...');

% calculate initial orientation
lg_q_prev = calc_lg_q_accelmag(data(1,:)); % previous orientation quat

disp('Initial orientation calculated.');
disp('Calculating orientations w/ gyro data...');

% choose selected orientation calculation function
calc_func = calc_lg_q;

lg_q = rowfun(calc_func, data);

disp('Orientations calculated.');
disp('Converting to Euler angles...');

ANGLES = ['Yaw', 'Pitch', 'Roll'];

% TODO: Convert Python code to MATLAB
lg_angles = varfun(@(x) x.yaw_pitch_roll, lg_q); % did not convert to array
lg_angles.Properties.VariableNames = ANGLES;
lg_angles = varfun(@(x) x * RAD_TO_DEG, lg_angles);
lg_angles('Time') = data('Time'); % May need to convert to array before assignment.

if NORM_HEADING
    heading_offset = mean(head(lg_angles('Yaw'), 48));
    lg_angles('Yaw') = varfun(@(x) x - heading_offset, lg_angles('Yaw'));
end

disp('Euler angles calculated.');

% Line 394 of kga_quat.py: is this needed?
plotter.draw_sensors('ea_kga');

disp('Saving Euler angles to ''out/ea_kga.csv''...');
writetable(lg_angles(['Roll', 'Pitch', 'Yaw']), 'out/ea_kga.csv', 'WriteRowNames', false, 'WriteVariableNames', false);
disp('Done.');

disp('Saving quats to ''out/quat_kga.csv''...');
lg_quat_arr = varfun(@(x) x.elements, lg_q); % did not convert to array
lg_quat_arr.Properties.VariableNames = ['w','x','y','z'];
lg_quat_arr.to_csv("out/quat_kga.csv", index=False, header=False);
writetable(lg_quat_arr, 'out/quat_kga.csv', 'WriteRowNames', false, 'WriteVariableNames', false);
disp('Done.');

disp('Loading orientation view...');

% TODO: Convert this Python code to MATLAB.
% Q: What does this code do?
% root = os.path.dirname(os.path.abspath(__file__))
% bin_path = os.path.join(root, "../bin")
% file_path = os.path.join(root, "../out/quat_kga.csv")
% 
% subprocess.run(["orientation_view", "-sps", "96", "-file", file_path], cwd=bin_path, shell=True)

%===========================================
% BEGIN KGA ALGORITHM FUNCTIONS (9 in total)
%===========================================

% previous orientation quat
%lg_q_prev = NaN; % not needed

% previous gyro vector
W_prev = [0, 0, 0];

% current gyro bias
w_bias = [0, 0, 0];

function lg_q_accelmag = cal_lg_q_accelmag(row)
    %{
    Calculates the quaternion representing the orientation
    of the global frame relative to the local frame,
    using only accel and mag calculations.

    Should be used to calculate the initial orientation.
    row - represents a row of the data table.
    %}

    % create normalized acceleration vector
    acc = table2array(row(ACC_COLS));
    acc = acc / normalize(acc);
    
    % create normalized magnetometer vector
    mag = table2array(row(MAG_COLS));
    mag = mag / normalize(mag);
    
    % calculate acceleration quat
    acc_args = num2cell(acc);
    q_acc = calc_q_acc(acc_args{:});
    
    % rotate mag vector into intermediate frame
    l_mag = quatrotate(quatinv(q_acc), mag); % need to use XIO quat lib
    
    % calculate mag quat
    lmag_args = num2cell(l_mag); % convert quat array to cell array
    q_mag = calc_q_mag(lmag_args{:});
    
    % combine quats (Equation 13)
    lg_q_accelmag = q_acc * q_mag;
    
    % only return q_mag if selected
    return;
end

function lg_q = calc_lg_q(row)
    %{
    Calculates the quaternion representing acceleration data, `q_acc` (Equation 25).

    The acceleration vector should be normalized before being passed into this function.
    %}

    % create normalized acceleration vector
    acc = table2array(row(ACC_COLS));
    acc_mag = normalize(acc);
    acc = acc / acc_mag;
    
    % create normalized magnetometer vector
    mag = table2array(row(MAG_COLS));
    mag = mag / normalize(mag);
    
    % create gyro vector and remove current bias
    gyro = table2array(row(GYRO_COLS));
    
    % update gyro bias calculation
    if UPDATE_GYRO_BIAS
        update_gyro_bias(acc_mag, gyro);
    end
    
    % correct for gyro bias
    gyro = gyro - w_bias;
    
    % calculate adaptive gain from acc if selected
    alpha = 0;
    if USE_ADAPTIVE_GAIN
        alpha = calc_gain(GAIN, acc_mag);
    else
        alpha = GAIN;
    end
    
    % calculate gyro quaternion
    gyro_args = num2cell(gyro);
    lg_q_w = calc_q_w(gyro_args{:});
    
    % rotate acc vector into frame
    g_pred = 0; % need to use XIO quat library
    
    % calculate acceleration quat
    g_pred_args = num2cell(g_pred);
    q_acc = calc_q_acc(g_pred_args{:});
    
    % TODO: LERP/SLERP q_acc
    q_acc_adj = scale_quat(alpha, q_acc);
    
    % calculate intermediate quat
    lg_q_prime = lg_q_w * q_acc_adj;
    
    % rotate mag vector into intermediate frame
    l_mag = 0; % need to use XIO quat library
    
    % calculate mag quat
    l_mag_args = num2cell(l_mag);
    q_mag = calc_q_mag(l_mag_args{:});
    
    % TODO: LERP/SLERP q_mag
    q_mag_adj = scale_quat(alpha, q_mag);
    
    % combin quats (Equation 13)
    lg_q_prev = lg_q_prime * q_mag_adj;
    
    % only return q_mag if selected
    lg_q = lg_q_prev;
    if ONLY_Q_MAG
        lg_q = q_mag;
    end
    
    return;
end

function qacc = calc_q_acc(ax, ay, az)
    %{
    Calculates the quaternion representing acceleration data, `q_acc` (Equation 25).

    The acceleration vector should be normalized before being passed into this function.
    %}
    
    [q0, q1, q2, q3] = deal(0,0,0,0);
    
    if az >= 0
        q0 = sqrt((az + 1) / 2);
        q1 = -ay / sqrt(2*(az + 1));
        q2 = ax / sqrt(2*(az + 1));
    else
        q0 = -ay / sqrt(2*(1 - az));
        q1 = sqrt((1 - az) / 2);
        q3 = ax / sqrt(2*(1 - az));
    end

    qacc = 0; % need to use XIO quat library
    
    return;
end

function qmag = calc_q_mag(mx, my, mz)
    %{
    Calculates the quaternion representing magnetometer data, `q_mag` (Equation 35).

    The magnetometer vector should be normalized and calibrated before being passed into this function.
    %}

    [q0, q1, q2, q3] = deal(0,0,0,0);

    % L represents gamma
    L = mx^2 + my^2;
    
    if mx >= 0
        q0 = sqrt(L + mx*sqrt(L)) / sqrt(2*L);
        q3 = my / (sqrt(2) * sqrt(L + mx*sqrt(L)));
    else
        q0 = my / (sqrt(2) * sqrt(L - mx*sqrt(L)));;
        q3 = sqrt(L - mx*sqrt(L)) / sqrt(2*L);
    end

    qmag = 0; % need to use XIO quat library
    
    return;
end

function qw = calc_q_w(wx, wy, wz)
    % Calculates the quaternion representing gyroscope data, `q_w` (Equation 42).
    
    % calculate delta gyro quaternion
    w_quat = 0; % need to use XIO quat library
    dq_w = (-1/2) * w_quat * lg_q_prev;
    
    % add delta gyro quat to previous orientation
    qw = lg_q_prev + dq_w * (1/96);
    
    return;
end

function adap_gain = calc_gain(alpha, a_mag)
    %{
    Calculates the adaptive gain for scaling correction quaternions.
    Will return a floating point number between 0 and 1.
    %}

    error = abs(a_mag - GRAVITY) / GRAVITY;
    
    [bound1, bound2] = deal(0.1, 0.2);
    m = 1.0/(bound1 - bound2);
    b = 1.0 - m * bound1;
    
    factor = 0.0;
    
    if error < bound1
        factor = 1.0;
    elseif error < bound2
        factor = m*error + b;
    end
    
    adap_gain = factor * alpha;
    
    return;
end

function res = is_steady_state(acc_mag, wx, wy, wz)
    % Checks if the module is in a steady state with no external dynamic motion or rotation
    res = true;
    
    % check if module is in nongravitational dynamic motion
    if abs(acc_mag - GRAVITY) > ACC_THRESHOLD
        res = false;
        return;
    end
    
    % check if module has changed angular acceleration
    if abs(wx - w_prev(1)) > DELTA_GYRO_THRESHOLD ...
            || abs(wy - w_prev(2)) > DELTA_GYRO_THRESHOLD ...
            || abs(wz - w_prev(3)) > DELTA_GYRO_THRESHOLD
        res = false;
        return;
    end
    
    % check if module is currently rotating
    if abs(wx - w_bias(1)) > GYRO_THRESHOLD ...
            || abs(wy - w_bias(2)) > GYRO_THRESHOLD ...
            || abs(wz - w_bias(3)) > GYRO_THRESHOLD
        res = false;
        return;
    end
    
    return;
end

function update_gyro_bias(acc_mag, w)
    % Calculates new gyro bias if the module is in a steady state.
    % This fn alters global variables.
end

function squat = scale_quat(gain, quat)
    %{
    Scales the given quaternion by an interpolation with the identity quaternion.
    Uses LERP or SLERP depending on the angle between the quaternion and identity quaternion.
    %}
    squat = 0;
    return;
end

%===========================================
% END KGA ALGORITHM FUNCTIONS
%===========================================