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

%===========================================
% BEGIN KGA ALGORITHM FUNCTIONS (9 in total)
%===========================================

% previous orientation quat
lg_q_prev = NaN;

% previous gyro vector
W_prev = [0, 0, 0];

% current gyro bias
w_bias = [0, 0, 0];

function lg_q = cal_lg_q_accelmag(row)
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
    l_mag = quatrotate(quatinv(q_acc), mag);
    
    % calculate mag quat
    lmag_args = num2cell(l_mag); % convert quat array to cell array
    q_mag = calc_q_mag(lmag_args{:});
    
    % combine quats (Equation 13)
    lg_q = q_acc * q_mag;
    
    % only return q_mag if selected
    return;
end

function res = calc_lg_q(row)
    %{
    Calculates the quaternion representing acceleration data, `q_acc` (Equation 25).

    The acceleration vector should be normalized before being passed into this function.
    %}
    res = 0;
    return;
end

function accq = calc_q_acc(ax, ay, az)
    %{
    Calculates the quaternion representing acceleration data, `q_acc` (Equation 25).

    The acceleration vector should be normalized before being passed into this function.
    %}
    accq = 0;
    return;
end

function magq = calc_q_mag(mx, my, mz)
    %{
    Calculates the quaternion representing magnetometer data, `q_mag` (Equation 35).

    The magnetometer vector should be normalized and calibrated before being passed into this function.
    %}
    magq = 0;
    return;
end

function gyroq = calc_q_w(wx, wy, wz)
    % Calculates the quaternion representing gyroscope data, `q_w` (Equation 42).
    gyroq = 0;
    return;
end

function adap_gain = calc_gain(alpha, a_mag)
    %{
    Calculates the adaptive gain for scaling correction quaternions.
    Will return a floating point number between 0 and 1.
    %}
    adap_gain = 0;
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
