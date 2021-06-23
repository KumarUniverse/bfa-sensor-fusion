
function [data, params] = csv_reader(test_name, same_sps, correct_axes, convert_to_rads, apply_gyro_bias)
    %{
    Reads raw test data from a given CSV and returns a MATLAB table.
    This method does NOT filter data or apply biases, only returning the readings for each sensor.
    %}
    addpath('src/lib')
    %import lib.constants.*
    load('constants.mat');
    
    switch nargin  % used in place of default function parameters
        case 1
            same_sps = false;
            correct_axes = false;
            convert_to_rads = false;
            apply_gyro_bias = false;
        case 2
            correct_axes = false;
            convert_to_rads = false;
            apply_gyro_bias = false;
        case 3
            convert_to_rads = false;
            apply_gyro_bias = false;
        case 4
            apply_gyro_bias = false;
        case 5
        otherwise
            warning('Not enough parameters provided. Need at least 1 and no more than 5')
    end
    
    % convert test_name to a path if it isn't already one
    if isfile(test_name) == 0  % if file does not exist
        test_name = strcat('data/', test_name, '.csv');
    end
    csv_file_path = test_name;
    
    % read test params from CSVP
    csvp_id = fopen(strcat(csv_file_path, 'p'), 'r');
    
    % create params array
    % Matlab advises against dynamically growing an array for performance reasons. May need to change.
    params = [];
    tline = fgetl(csvp_id);
    while ischar(tline)
        params(end+1) = (str2double(tline));  % appending to array can be expensive
        % params(end+1) = (str2double(['uint8(',tline,')'])); % probably not needed
        tline = fgetl(csvp_id);
    end
    fclose(csvp_id);
    
    % read data from CSV
    % For some reason, MATLAB automatically excludes the first row.
    opts = detectImportOptions(csv_file_path);
    opts.VariableNamesLine = 0; % 0 means don't import variable names
    opts.VariableNames = AXES;
    data = readtable(csv_file_path, opts);
    %data.Properties.VariableNames = AXES;
    %disp(data(1:10,:)); % glimpse the data
    %disp(size(data));
    
    sample_rate = params(8);
    
    % add time axis to dataset
    len_data = height(data);
    %disp(len_data); % 46740
    %disp(sample_rate); % 960
    time_col = array2table(transpose(0:1/sample_rate:(len_data-1)/sample_rate));
    time_col.Properties.VariableNames = {'Time'};
    % insert time column to the beginning of the data table
    %disp(size(time_col)); % 46740 x 1 remove after debugging
    %disp(size(data)); % 46740 x 9 remove after debugging
    data = [time_col data]; % ERROR: All tables being horizontally concatenated must have the same number of rows.
    
    % sign data
    var_names = data.Properties.VariableNames;
    data = varfun(@(x) sign_data(x), data); % variable*scalar_fn
    data.Properties.VariableNames = var_names;

    % apply accel sensitivity
    acc_sens = params(10);
    
    apply_accel_sens = @(x) x * acc_sens * GRAVITY / 32768; %apply_accel_sens_decorator(acc_sens, GRAVITY);
    var_names = data.Properties.VariableNames;
    data(:,ACC_COLS) = varfun(apply_accel_sens, data(:,ACC_COLS));
    data.Properties.VariableNames = var_names;
    
    % invert Y and Z for accel and gyro
    % TODO: not sure if this is necessary
    % data[["AccelY", "AccelZ"]] = -data[["AccelY", "AccelZ"]]
    % data[["GyroY", "GyroZ"]] = -data[["GyroY", "GyroZ"]]
    
    % calculate conversion factor if selected
    GYRO_UNITS = 0;
    if convert_to_rads
        GYRO_UNITS = DEG_TO_RAD;
    else
        GYRO_UNITS = 1;
    end
    
    % apply gyro sensitivity
    gyro_sens = params(11);
    
    apply_gyro_sens = @(x) x * gyro_sens * GYRO_UNITS / 32768; %apply_gyro_sens_decorator(GYRO_UNITS, gyro_sens);
    var_names = data.Properties.VariableNames;
    data(:,GYRO_COLS) = varfun(apply_gyro_sens, data(:,GYRO_COLS));
    data.Properties.VariableNames = var_names;
    
    % apply mag sensitivity
    mag_sens = 4800;
    
    apply_mag_sens = @(x) x * mag_sens / 32768; %apply_mag_sens_decorator(mag_sens);
    var_names = data.Properties.VariableNames;
    data(:,MAG_COLS) = varfun(apply_mag_sens, data(:,MAG_COLS));
    data.Properties.VariableNames = var_names;
    
    % calculate gyro bias using first 0.5s of data.
    data_head = head(data, 480);
    gyro_offsets = mean(table2array(data_head(:,GYRO_COLS)), 'omitnan');

    % apply offsets to gyroscope (remove sensor bias)
    for i = 1:size(GYRO_COLS, 2)
        axis = GYRO_COLS(i);
        var_names = data.Properties.VariableNames;
        data(:,axis) = varfun(@(x) x - gyro_offsets(i), data(:,axis));
        data.Properties.VariableNames = var_names;
    end

    % if selected, manipulate axes to align mag with accel/gyro axes
    if correct_axes
        data = movevars(data, 'MagY', 'After', 'MagX');
        data = movevars(data, 'MagX', 'After', 'GyroY');
        data(:, 'MagZ') = array2table(-table2array(data(:,'MagZ')));
    end
    
%     FIXME: experimental mag modifications, very very incorrect
%     new_mag_data = data[MAG_COLS]
%     new_mag_data["MagX"] = -data["MagZ"]
%     new_mag_data["MagY"] = data["MagX"]
%     new_mag_data["MagZ"] = -data["MagY"]
%     data[MAG_COLS] = new_mag_data
%     data[MAG_COLS] = data[MAG_COLS].values[::-1]

    % reorder axes so that mag columns are in X-Y-Z order
    %data = data('Time' + AXES);
    data.Properties.VariableNames{'MagY'} = 'temp';
    data.Properties.VariableNames{'MagX'} = 'MagY';
    data.Properties.VariableNames{'temp'} = 'MagX';
    
    % fill null mag values with previous value
    data = fillmissing(data, 'previous');
    
    % if enabled, only include every 10th row
    % to create 96sps data (downsampling)
    if same_sps
        data = data(10:10:end,:);
        params(8) = params(8) / 10;
    end
    
    % for some reason, the first mag data point is always erroneous,
    % so remove its row
    % Not needed since the first row is automatically excluded by MATLAB.
    %data(1,:) = [];
    
    return;
end

function y = sign_data(x)
    % signs the raw unsigned data.
    if x > 32767
        y = x-65535;
    else
        y = x;
    end

    return;
end
