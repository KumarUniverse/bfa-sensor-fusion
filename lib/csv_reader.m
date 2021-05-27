import lib.constants

function [data, params] = read(test_name, same_sps, correct_axes, convert_to_rads, apply_gyro_bias)
    %{
    Reads raw test data from a given CSV and returns a Pandas DataFrame.
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
    params = [];
    tline = fgetl(csvp_id);
    while ischar(tline)
        params(end+1) = (str2double(['uint8(',tline,')']));  % appending to array can be expensive
        tline = fgetl(csvp_id);
    end
    fclose(csvp);
    
    % read data from CSV
    data = readtable(csv_file_path);
    data.Properties.VariableNames = constants.AXES;
    
    sample_rate = params(8);
    
    % add time axis to dataset
    len_data = height(data);
    time_col = table(0:1/sample_rate:(len_data/sample_rate - 1));
    time_col.Properties.VariableNames = {'Time'};
    % insert time column to the beginning of the data table
    data = [time_col data];
    
    % sign data
    %data = data.applymap(lambda x: x-65535 if x > 32767 else x)
    % Note: nested for loop is more computationally expensive and
    % time consuming compared to a scalar update of the data.
    for i = 1:size(data, 1)
        for j = 1:size(data, 2)
            prev_data = data(i,j);
            if prev_data > 32767
                data(i,j) = prev_data - 65535;
            end
        end
    end
    % attempt at scalar update (does not work):
    %var_names = data.Properties.VariableNames;
    %data = varfun(@(x) (if x > 32767 1; else 0; end), data); % variable*scalar_fn
    %data.Properties.VariableNames = var_names;

    % apply accel sensitivity
    acc_sens = params(10);
    %data(constants.ACC_COLS) = data(constants.ACC_COLS).applymap(lambda x: x * acc_sens * GRAVITY / 32768)
end

%{
def read(test_name: str, same_sps=False, correct_axes=False, convert_to_rads=False, apply_gyro_bias=False) -> pd.DataFrame:
    """
    Reads raw test data from a given CSV and returns a Pandas DataFrame.
    
    This method does NOT filter data or apply biases, only returning the readings for each sensor.
    """

    # convert test_name to a path if it isn't already one
    file_path = test_name if path.exists(test_name) else f"data/{test_name}.csv"

    # read test params from CSVP
    csvp = open(file_path + "p")

    # create params array
    params = np.array([eval(line) for line in csvp])

    # read data from CSV 
    data = pd.read_csv(file_path, names=AXES, index_col=False)

    sample_rate = params[7]

    # add time axis to data set
    time = np.arange(0, len(data)/sample_rate, 1/sample_rate)
    data.insert(0, "Time", time)

    # sign data
    data = data.applymap(lambda x: x-65535 if x > 32767 else x)

    # apply accel sensitivity
    acc_sens = params[9]
    data[ACC_COLS] = data[ACC_COLS].applymap(lambda x: x * acc_sens * GRAVITY / 32768)

    # invert Y and Z for accel and gyro
    # TODO: not sure if this is necessary
    # data[["AccelY", "AccelZ"]] = -data[["AccelY", "AccelZ"]]
    # data[["GyroY", "GyroZ"]] = -data[["GyroY", "GyroZ"]]

    # calculate conversion factor if selected
    GYRO_UNITS = DEG_TO_RAD if convert_to_rads else 1

    # apply gyro sensitivity
    gyro_sens = params[10]
    data[GYRO_COLS] = data[GYRO_COLS].applymap(lambda x: x * gyro_sens * GYRO_UNITS / 32768)

    # apply mag sensitivity
    mag_sens = 4800
    data[MAG_COLS] = data[MAG_COLS].applymap(lambda x: x * mag_sens / 32768)
        
    # calculate gyro bias using first 0.5s of data
    gyro_offsets = data[GYRO_COLS].head(480).mean()

    # apply offsets to gyroscope (remove sensor bias)
    for i, axis in enumerate(GYRO_COLS):
        data[axis] = data[axis].map(lambda x: x - gyro_offsets[i])

    # if selected, manipulate axes to align mag with accel/gyro axes
    if correct_axes:
        data[["MagX","MagY"]] = data[["MagY","MagX"]]
        data["MagZ"] = -data["MagZ"]

    # FIXME: experimental mag modifications, very very incorrect
    # new_mag_data = data[MAG_COLS]
    # new_mag_data["MagX"] = -data["MagZ"]
    # new_mag_data["MagY"] = data["MagX"]
    # new_mag_data["MagZ"] = -data["MagY"]
    # data[MAG_COLS] = new_mag_data
    # data[MAG_COLS] = data[MAG_COLS].values[::-1]

    # reorder axes so that mag columns are in X-Y-Z order
    data = data[["Time"] + AXES]
    
    #fill null mag values with previous value
    data = data.fillna(method='ffill')

    # if enabled, remove every 10th row to create 96sps data
    if same_sps: 
        data = data.iloc[::10]
        params[7] /= 10

    # for some reason, the first mag data point is always erroneous, so remove its row
    data = data.iloc[1:]

    return data, params
%}