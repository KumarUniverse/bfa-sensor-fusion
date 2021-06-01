import lib.constants.*


function [data, params] = read(test_name, same_sps, correct_axes, convert_to_rads, apply_gyro_bias)
    %{
    Reads raw test data from a given CSV and returns a MATLAB table.
    This method does NOT filter data or apply biases, only returning the readings for each sensor.
    %}
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
        test_name = "data" + test_name + ".csv";
    end
    csv_file_path = test_name;
    
    % read test params from CSVP
    csvp_id = fopen(csv_file_path + "p", 'r');
    
    % create params array
    % Matlab advises against dynamically growing an array for performance reasons. May need to change.
    params = [];
    tline = fgetl(csvp_id);
    while ischar(tline)
        params(end+1) = (str2double(['uint8(',tline,')']));  % appending to array can be expensive
        tline = fgetl(csvp_id);
    end
    fclose(csvp);
    
    % read data from CSV
    data = readtable(csv_file_path);
    data.Properties.VariableNames = AXES;
    
    sample_rate = params(8);
    
    % add time axis to dataset
    len_data = height(data);
    time_col = table(0:1/sample_rate:(len_data/sample_rate - 1));
    time_col.Properties.VariableNames = {'Time'};
    % insert time column to the beginning of the data table
    data = [time_col data];
    
    % sign data
    function y = sign_data(x)
        % signs the raw unsigned data.
        if x > 32767
            y = x-65535;
        else
            y = x;
        end

        return;
    end
    %data = data.applymap(lambda x: x-65535 if x > 32767 else x)
    % Note: nested for loop is more computationally expensive and
    % time consuming compared to a scalar update of the data.
%     for i = 1:size(data, 1)
%        for j = 1:size(data, 2)
%            prev_data = data(i,j);
%            if prev_data > 32767
%                data(i,j) = prev_data - 65535;
%            end
%        end
%     end
    % attempt at scalar update:
    var_names = data.Properties.VariableNames;
    data = varfun(sign_data(x), data); % variable*scalar_fn
    data.Properties.VariableNames = var_names;

    % apply accel sensitivity
    acc_sens = params(10);
    function y = apply_accel_sens(x)
        % Applies acceleration sensitivity to the raw acceleration data.
        y = x * acc_sens * GRAVITY / 32768;

        return;
    end
    var_names = data.Properties.VariableNames;
    data(ACC_COLS) = varfun(apply_accel_sens, data(ACC_COLS));
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
    function y = apply_gyro_sens(x)
        % Applies gyroscopic sensitivity to the raw gyro data.
        y = x * gyro_sens * GYRO_UNITS / 32768;

        return;
    end
    var_names = data.Properties.VariableNames;
    data(GYRO_COLS) = varfun(apply_gyro_sens, data(GYRO_COLS));
    data.Properties.VariableNames = var_names;
    
    % apply mag sensitivity
    mag_sens = 4800;
    function y = apply_mag_sens(x)
        % Applies magnetic sensitivity to the raw mag data.
        y = x * mag_sens / 32768;

        return;
    end
    var_names = data.Properties.VariableNames;
    data(MAG_COLS) = varfun(apply_mag_sens, data(MAG_COLS));
    data.Properties.VariableNames = var_names;
    
    % calculate gyro bias using first 0.5s of data.
    data_temp = head(data, 480);
    gyro_offsets = mean(data_temp(GYRO_COLS));
    
    % apply offsets to gyroscope (remove sensor bias)
    for i = 1:size(GYRO_COLS, 2)
        axis = GYRO_COLS(i);
        var_names = data.Properties.VariableNames;
        data(axis) = varfun(@(x) x - gyro_offsets(i), data(axis));
        data.Properties.VariableNames = var_names;
    end
    
    % if selected, manipulate axes to align mag with accel/gyro axes
    if correct_axes
        data('MagX','MagY') = data('MagY','MagX');
        data('MagZ') = -data('MagZ');
    end
    
%     FIXME: experimental mag modifications, very very incorrect
%     new_mag_data = data[MAG_COLS]
%     new_mag_data["MagX"] = -data["MagZ"]
%     new_mag_data["MagY"] = data["MagX"]
%     new_mag_data["MagZ"] = -data["MagY"]
%     data[MAG_COLS] = new_mag_data
%     data[MAG_COLS] = data[MAG_COLS].values[::-1]

    % reorder axes so that mag columns are in X-Y-Z order
    data = data('Time' + AXES);
    
    % fill null mag values with previous value
    data = fillmissing(data, 'previous');
    
    % if enabled, remove every 10th row to create 96sps data (downsampling)
    if same_sps
        data(10:10:end,:) = [];
        params(8) = params(8) / 10;
    end
    
    % for some reason, the first mag data point is always erroneous,
    % so remove its row
    data(1,:) = [];
    
    return;
end
