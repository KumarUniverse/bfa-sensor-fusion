
function cal_mag_data = magcal_calibrate(mag_data, M, n, d)
    %{
    Returns a calibrated set of magnetometer samples.

    `mag_data` should be passed as a MATLAB table with columns `[MagX, MagY, MagZ]`.
    %}
    % Note: mag_data parameter is assumed to be a MATLAB table,
    % not an array. Return type is a MATLAB table.
    
    % calculate ellipsoid parameters if not passed in
    disp('num of args: ');
    disp(nargin); % 1
    disp(mag_data(1:5,:));
    disp(M); % Not enough input args.
    disp(n);
    disp(d);
    if nargin == 1  % if only 1 arg is passed to the function
        [M, n, d] = ellipsoid_fit(table2array(mag_data)');
    end
    
    disp(M, n, d);
    
    % calculate calibration parameters for:
    % h_m = A @ h + b where h = A^-1 @ (h_m - b)
    M_1 = inv(M);
    b = dot(M_1, n);
    % sqrtm - matrix square root
    A_1 = real(1 / sqrt(dot(n.T, dot(M_1, n)) - d) * sqrtm(M));
    
    % apply calibration to mag samples and return calibrated data.
    % Concern: Not sure if this is equivalent to pandas.apply().
    % original Python code: mag_data.apply(__calibrate_row, args=(A_1, b), axis=1, result_type='expand')
    cal_mag_data = rowfun(private_calibrate_row, mag_data, A_1, b);

    return;
end

function res = private_calibrate_row(row, A_1, b)
    %{
    Internal method to calculate correct magnetometer reading at a specific time.

    Assumes that `row` = `[MagX, MagY, MagZ]`.
    Uses correction equation `h = A^-1 @ (h_m - b)`. 
    %}
    res = A_1 * (cat(1, row)' - b); % not sure if this is correct.
    res = res(:);
    
    return;
end

