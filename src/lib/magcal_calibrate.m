
function cal_mag_data = magcal_calibrate(mag_data, M, n, d, first, last)
    %{
    Returns a calibrated set of magnetometer samples.

    `mag_data` should be passed as a MATLAB table with columns `[MagX, MagY, MagZ]`.
    %}
    % Note: mag_data parameter is assumed to be a MATLAB table,
    % not an array. Return type is a MATLAB table.
    addpath('src/lib'); % KGA library
    
    % calculate ellipsoid parameters if not passed in
    if nargin == 1
        [M, n, d] = magcal_ellipsoid_fit(table2array(mag_data).');
    elseif nargin == 4 && (isnan(M) || isnan(n) || isnan(d))
        [M, n, d] = magcal_ellipsoid_fit(table2array(mag_data).');
    elseif nargin == 6 && (isnan(M) || isnan(n) || isnan(d))
        if isnan(first)
            first = 1;
        end
        if isnan(last)
            last = height(mag_data);
        end
        mag_data_clipped = mag_data(first:last,:);
        disp(head(mag_data_clipped, 5));
        [M, n, d] = magcal_ellipsoid_fit(table2array(mag_data_clipped).');
    end
    
    % display M, n, and d
    disp('M');
    disp(M);
    disp('n');
    disp(n);
    disp('d');
    disp(d);
    % display mag_data
    disp(head(mag_data, 5));
    
    % calculate calibration parameters for:
    % h_m = A @ h + b where h = A^-1 @ (h_m - b)
    M_1 = inv(M);
    global A_1 b;
    b = -(M_1 * n);
    % sqrtm - matrix square root
    A_1 = real(1 / sqrt((n.' * (M_1 * n)) - d) * sqrtm(M));
    
    % apply calibration to mag samples and return calibrated data.
    % Concern: Not sure if this is equivalent to pandas.apply().
    % original Python code: mag_data.apply(__calibrate_row, args=(A_1, b), axis=1, result_type='expand')
    cal_mag_data = rowfun(@private_calibrate_row, mag_data);
    cal_mag_data = array2table(table2array(cal_mag_data)); % split into columns
    cal_mag_data.Properties.VariableNames = mag_data.Properties.VariableNames;

    return;
end

function res = private_calibrate_row(varargin)
    %{
    Internal method to calculate correct magnetometer reading at a specific time.

    Assumes that `row` = `[MagX, MagY, MagZ]`.
    Uses correction equation `h = A^-1 @ (h_m - b)`. 
    %}
    
    global A_1 b;
    row = cell2mat(varargin);
    res = A_1 * (row.' - b); % not sure if this is correct
    res = res.';

    return;
end

