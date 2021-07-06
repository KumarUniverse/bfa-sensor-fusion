% Keeping a Good Attitude (KGA) Algorithm
addpath('src/lib'); % KGA library
addpath('src/quaternion_library');  % include quaternion library
global constants;
constants = load('constants.mat');

%======================================

TEST_NAME = 'euler_angles_2';

% booleans:
% Note: Only those booleans that were found to be necessary in
% kga_quat.py are used in this script.
global APPLY_SMOOTHING CALIBRATE_MAG USE_PRECALC_MAG CORRECT_MAG_AXES ...
    NORM_HEADING;
APPLY_SMOOTHING = NaN;   % "BUTTER", "SMA", None to disable
CALIBRATE_MAG = true;    % should be disabled if mag data is already calibrated
USE_PRECALC_MAG = false; % uses hard-coded mag calibration parameters
CORRECT_MAG_AXES = true; % re-aligns mag axes to match accel/gyro axes (needed for MPU-9250 data)
NORM_HEADING = true;     % normalizes yaw in euler angles graph (cosmetic, does not affect calculations)

% KGA complementary filter parameters:
global GAIN BIAS_ALPHA GYRO_THRESHOLD ACC_THRESHOLD ...
    DELTA_GYRO_THRESHOLD USE_ADAPTIVE_GAIN UPDATE_GYRO_BIAS;
GAIN = 0.01;
BIAS_ALPHA = 0.01;
GYRO_THRESHOLD = 0.2;
ACC_THRESHOLD = 0.1;
DELTA_GYRO_THRESHOLD = 0.1;

USE_ADAPTIVE_GAIN = true;
UPDATE_GYRO_BIAS = true;
%======================================
% Hard-coded mag parameters for "euler_angles_2"
% (intended to be used with `USE_PRECALC_MAG`)

M = [0.56144721 -0.01910871 -0.01292889; ...
     -0.01910871  0.6276801  -0.00568568; ...
     -0.01292889 -0.00568568  0.53873008];
     
n = [-13.60233683; -1.60856291; 11.10481335];
    
d = -390.59292573690266;

%======================================
% SEE ALSO: KGA C++ implementation, published by the authors of the original paper
% https://github.com/ccny-ros-pkg/imu_tools/blob/indigo/imu_complementary_filter/src/complementary_filter.cpp
disp('KGA algorithm started.');
disp(['Reading test ', TEST_NAME, '...']);

% read test data at 96 samples/second and convert gyro data to rads
% [data, params] = read(TEST_NAME, 'same_sps', true, ...
%     'correct_axes', CORRECT_MAG_AXES, 'convert_to_rads', true, ...
%     'apply_gyro_bias', true);
[data, params] = csv_reader(TEST_NAME, true, CORRECT_MAG_AXES, true, true);
% disp('First 10 lines of sensor data:'); % remove after debugging
% disp(data(1:10,:)); % this is correct

ORDER = 10;
CUTOFF_FREQ = 50;
SAMPLE_RATE = 960;
NORM_CUTOFF_FREQ = CUTOFF_FREQ / (2 * SAMPLE_RATE);

% Note: Smoothing is not applied to the acc data.

disp('Test read.');

% TODO: is this needed?
if CALIBRATE_MAG
    disp('Calibrating magnetometer data...');

    if USE_PRECALC_MAG
        data(:,constants.MAG_COLS) = magcal_calibrate(data(:,constants.MAG_COLS), M, n, d);
    else
        data(:,constants.MAG_COLS) = magcal_calibrate(data(:,constants.MAG_COLS));
    end
    
    disp('Magnetometer calibration complete.');
end

% DEBUG code not included
% Moved lines 164-176 to here:
ANGLES = {'Yaw', 'Pitch', 'Roll'};

global lg_q_prev w_prev w_bias;

% previous orientation quat
lg_q_prev = NaN;

% previous gyro vector
w_prev = [0, 0, 0];

% current gyro bias
w_bias = [0, 0, 0];

disp('Calculating initial orientation...');

% calculate initial orientation
lg_q_prev = calc_lg_q_accelmag(data(1,:)); % previous orientation quat

disp('Initial orientation calculated.');
disp('Calculating orientations w/ gyro data...');

% lg_q_first_row = calc_lg_q(data(1,2:end)); % REMOVE after debugging
% disp('lg_q_first_row');
% disp(lg_q_first_row);
lg_q = rowfun(@calc_lg_q, data(:,2:end));
%lg_q = array2table(table2array(lg_q)); % split into multiple columns
%^^may not be needed
%disp('lg_q'); % remove after debugging
%disp(lg_q(1:5,:)); % this seems to be wrong

disp('Orientations calculated.');
disp('Converting to Euler angles...');

lg_angles = rowfun(@(x) -flip(quatern2euler(x / norm(x))), lg_q);
%lg_angles = rowfun(@(x) quatern2eulerYPR(x / norm(x)), lg_q); % equivalent to the above line
lg_angles = rowfun(@(x) x * constants.RAD_TO_DEG, lg_angles);
lg_angles = array2table(table2array(lg_angles)); % split the columns
lg_angles.Properties.VariableNames = ANGLES;
lg_angles(:,'Time') = data(:,'Time'); % May need to convert to array before assignment.

if NORM_HEADING
    heading_offset = mean(table2array(head(lg_angles(:,'Yaw'), 48)));
    %heading_offset % 53.4963. In Python: 54.5323
    lg_angles(:,'Yaw') = rowfun(@(x) x - heading_offset, lg_angles(:,'Yaw'));
end

% disp('lg_angles'); % remove after debugging
% disp(lg_angles(1:5,:));

disp('Euler angles calculated.');

% Line 394 of kga_quat.py: is this needed?
% ===================== UNCOMMENT THIS ===========================
% plotter.draw_euler_angles_xio('ea_kga');
% 
% disp('Saving Euler angles to ''out/ea_kga.csv''...');
% writetable(lg_angles(['Roll', 'Pitch', 'Yaw']), 'out/ea_kga.csv', 'WriteRowNames', false, 'WriteVariableNames', false);
% disp('Done.');
% 
% disp('Saving quats to ''out/quat_kga.csv''...');
% lg_quat_arr = varfun(@(x) x.elements, lg_q); % did not convert to array
% lg_quat_arr.Properties.VariableNames = ['w','x','y','z'];
% lg_quat_arr.to_csv('out/quat_kga.csv', index=False, header=False);
% writetable(lg_quat_arr, 'out/quat_kga.csv', 'WriteRowNames', false, 'WriteVariableNames', false);
% disp('Done.');
% 
% % TODO: Convert this Python code to MATLAB.
% % This is a way to view the results in 3D instead
% % of just roll/pitch/yaw graphs.
% disp('Loading orientation view...');
% 
% [root, ~, ~] = fileparts(matlab.desktop.editor.getActiveFilename);
% bin_path = fullfile(root, '../bin');
% file_path = fullfile(root, '../out/quat_kga.csv');
% 
% system(['cd ' bin_path ' && ' 'orientation_view ' '-sps ' '96 ' '-file ' file_path]);
% ===================== UNCOMMENT THIS ===========================
% Note: system() starts a new shell process to execute the command.

%===========================================
% BEGIN KGA ALGORITHM FUNCTIONS (9 in total)
%===========================================

% global lg_q_prev w_prev w_bias; % may not be needed

% % previous orientation quat
% lg_q_prev = NaN;

% % previous gyro vector
% w_prev = [0, 0, 0];
% 
% % current gyro bias
% w_bias = [0, 0, 0];
% disp('w_bias');
% disp(w_bias);
% disp('after w_bias');

function lg_q_accelmag = calc_lg_q_accelmag(row)
    %{
    Calculates the quaternion representing the orientation
    of the global frame relative to the local frame,
    using only accel and mag calculations.

    Should be used to calculate the initial orientation.
    row - represents a row of the data table.
    %}
    global constants;

    % create normalized acceleration vector
    acc = table2array(row(:,constants.ACC_COLS));
    acc = acc / norm(acc);
    
    % create normalized magnetometer vector
    mag = table2array(row(:,constants.MAG_COLS));
    mag = mag / norm(mag);
    
    % calculate acceleration quat
    acc_args = num2cell(acc);
    q_acc = calc_q_acc(acc_args{:});
    
    % rotate mag vector into intermediate frame
    quad_mag = cat(2, 0.0, mag);
    l_mag = quaternProd(quaternProd(quaternConj(q_acc / norm(q_acc)), quad_mag), q_acc);
    % l_mag = quatrotate(quatinv(q_acc), mag); % need to remove
    
    % convert l_mag from quat to Euler
    %l_mag_euler = quatern2euler(l_mag / norm(l_mag)); % this is wrong
    l_mag_euler = l_mag(2:end);
    lmag_args = num2cell(l_mag_euler); % convert quat array to cell array
    % ^^ignore real part of quat array l_mag when converting to cell array
    % calculate mag quat
    q_mag = calc_q_mag(lmag_args{:});
    
    % combine quats (Equation 13 of KGA paper, quat product)
    lg_q_accelmag = quaternProd(q_acc, q_mag); % incorrect dimensions for matrix mult.
    
    % only return q_mag if selected
    return;
end

function lg_q = calc_lg_q(varargin)
    %{
    Calculates the quaternion representing acceleration data, `q_acc` (Equation 25).

    The acceleration vector should be normalized before being passed into this function.
    %}

    global constants lg_q_prev w_bias ...
        GAIN UPDATE_GYRO_BIAS USE_ADAPTIVE_GAIN;
    row = cell2table(varargin); % (UNCOMMENT after debugging!)
    row.Properties.VariableNames = constants.AXES;

    % create normalized acceleration vector
    acc = table2array(row(:,constants.ACC_COLS));
    acc_mag = norm(acc); % this is WRONG: acc / norm(acc);
    acc = acc / acc_mag;
    acc = cat(2, 0.0, acc); % make acc a quaternion
    
    % create normalized magnetometer vector
    mag = table2array(row(:,constants.MAG_COLS));
    mag = mag / norm(mag);
    mag = cat(2, 0.0, mag); % make mag a quaternion
    
    % create gyro vector and remove current bias
    gyro = table2array(row(:,constants.GYRO_COLS));

    % update gyro bias calculation
    if UPDATE_GYRO_BIAS
        %acc_mag
        %gyro
        update_gyro_bias(acc_mag, gyro);
    end

    % correct for gyro bias
    gyro = gyro - w_bias;
%     disp('gyro after correcting gyro bias'); % remove after debugging
%     disp(gyro);
    
    % calculate adaptive gain from acc if selected
    alpha = GAIN;
    if USE_ADAPTIVE_GAIN % (UNCOMMENT after debugging!)
        alpha = calc_gain(GAIN, acc_mag);
    end
    
    % calculate gyro quaternion
    gyro_args = num2cell(gyro);
    lg_q_w = calc_q_w(gyro_args{:}); % (UNCOMMENT after debugging!)
    %lg_q_w = [0 1 2 3]; % remove after debugging
    
    % rotate acc vector into frame
    g_pred = quaternProd(quaternProd(lg_q_w, acc), quaternConj(lg_q_w / norm(lg_q_w)));

    
    % convert g_pred from quaternion to Euler angles
    g_pred_euler = g_pred(2:end);
    %g_pred_euler
    g_pred_args = num2cell(g_pred_euler);
    % calculate acceleration quat
    q_acc = calc_q_acc(g_pred_args{:}); % (UNCOMMENT after debugging!)
    %q_acc = [0 1 2 3]; % remove after debugging
    
    % TODO: LERP/SLERP q_acc
    q_acc_adj = scale_quat(alpha, q_acc);
    
    % calculate intermediate quat
    lg_q_prime = quaternProd(lg_q_w, q_acc_adj);
    %lg_q_prime = lg_q_w * q_acc_adj; % not needed (incorrect)
    
    % rotate mag vector into intermediate frame
    l_mag = quaternProd(quaternProd(lg_q_prime, mag), ...
        quaternConj(lg_q_prime / norm(lg_q_prime)));
    
    % convert l_mag from quat to Euler
    l_mag_euler = l_mag(2:end);
    %l_mag_euler
    l_mag_args = num2cell(l_mag_euler);
    % calculate mag quat
    q_mag = calc_q_mag(l_mag_args{:}); % (UNCOMMENT after debugging!)
    %q_mag = [0 1 2 3]; % remove after debugging
    
    % TODO: LERP/SLERP q_mag
    q_mag_adj = scale_quat(alpha, q_mag); % (UNCOMMENT after debugging!)
    %q_mag_adj = q_mag; % remove after debugging
    
    % combine quats (Equation 13)
    lg_q_prev = quaternProd(lg_q_prime, q_mag_adj);
    %lg_q_prev = lg_q_prime * q_mag_adj; % not needed (incorrect)
    
    lg_q = lg_q_prev;
    
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
    elseif az < 0
        q0 = -ay / sqrt(2*(1 - az));
        q1 = sqrt((1 - az) / 2);
        q3 = ax / sqrt(2*(1 - az));
    end

    qacc = [q0 q1 q2 q3];
    
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
    elseif mx < 0
        q0 = my / (sqrt(2) * sqrt(L - mx*sqrt(L)));
        q3 = sqrt(L - mx*sqrt(L)) / sqrt(2*L);
    end

    qmag = [q0 q1 q2 q3];
    
    return;
end

function qw = calc_q_w(wx, wy, wz)
    % Calculates the quaternion representing gyroscope data, `q_w` (Equation 42).
    global lg_q_prev;
    
    % calculate delta gyro quaternion
    w_quat = [0 wx wy wz];
    dq_w = (-1/2) * quaternProd(w_quat, lg_q_prev);
    %dq_w = (-1/2) * w_quat * lg_q_prev; % WRONG
    
    % add delta gyro quat to previous orientation
    qw = lg_q_prev + dq_w * (1/96);
    
    return;
end

function adap_gain = calc_gain(alpha, a_mag) % WORKS!
    %{
    Calculates the adaptive gain for scaling correction quaternions.
    Will return a floating point number between 0 and 1.
    %}
    global constants;

    error = abs(a_mag - constants.GRAVITY) / constants.GRAVITY;
    
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

function res = is_steady_state(acc_mag, wx, wy, wz) % WORKS!
    % Checks if the module is in a steady state with no external dynamic motion or rotation
    global constants w_prev w_bias ...
        ACC_THRESHOLD DELTA_GYRO_THRESHOLD GYRO_THRESHOLD;
    
    res = true;
    
    % check if module is in nongravitational dynamic motion
    if abs(acc_mag - constants.GRAVITY) > ACC_THRESHOLD
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

function update_gyro_bias(acc_mag, w) % WORKS!
    % Calculates new gyro bias if the module is in a steady state.
    % This fn alters global variables.
    
    global w_bias w_prev BIAS_ALPHA;
    
    w_args = num2cell(w);
    
    if is_steady_state(acc_mag, w_args{:})
        w_bias = BIAS_ALPHA * (w - w_bias);
        %disp(['Module at rest, updating gyro bias: ' w_bias]);
        disp(strcat("Module at rest, updating gyro bias: ", ...
            strjoin(string(w_bias))));
    end
    
    % update previous gyro calculation
    w_prev = w;
end

function squat = scale_quat(gain, quat) % WORKS!
    %{
    Scales the given quaternion by an interpolation with the identity quaternion.
    Uses LERP or SLERP depending on the angle between the quaternion and identity quaternion.
    %}
    
    [q0, q1, q2, q3] = deal(0,0,0,0);
    
    % LERP (to be more efficient):
    if quat(1) > 0.9
        q0 = (1 - gain) + gain * quat(1);
        q1 = gain * quat(2);
        q2 = gain * quat(3);
        q3 = gain * quat(4);
    else % SLERP
        angle = acos(quat(1));
        A = sin(angle * (1 - gain)) / sin(angle);
        B = sin(angle * gain) / sin(angle);
        
        q0 = A + B * quat(1);
        q1 = B * quat(2);
        q2 = B * quat(3);
        q3 = B * quat(4);
    end
    
    squat = [q0 q1 q2 q3];
    squat = squat / norm(squat); % normalize the scaled quat
    
    return;
end

%===========================================
% END KGA ALGORITHM FUNCTIONS
%===========================================