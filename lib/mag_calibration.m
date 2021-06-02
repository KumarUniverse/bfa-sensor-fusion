% sqrtm - matrix square root

% function t = transpose_table(input_table)
%     % A function used to transpose a MATLAB table.
%     % Potential alternative: rows2vars() (may not work as intended)
%     arr = table2array(input_table);
%     t = array2table(arr.');
%     
%     return;
% end

function [M, n, d] = ellipsoid_fit(s)
    %{
    Calculates ellipsoid parameters to normalize magnetometer data.
    
    From: https://teslabs.com/articles/magnetometer-calibration/
    %}
    % D (samples)
    D = [s(1)^2., s(2)^2., s(3)^2., ...
         2.*s(2)*s(3), 2.*s(1)*s(3), 2.*s(1)*s(2), ...
         2.*s(1), 2.*s(2), 2.*s(3), ones(size(s(1)))];
    
    % S, S_11, S_12, S_21, S_22 (eq. 11)
    S = dot(D, D');
    S_11 = S(1:6, 1:6);
    S_12 = S(1:6, 7:end);
    S_21 = S(7:end, 1:6);
    S_22 = S(7:end, 7:end);
    
    % C (Eq. 8, k=4)
    C = [-1  1  1  0  0  0; ...
          1 -1  1  0  0  0; ...
          1  1 -1  0  0  0; ...
          0  0  0 -4  0  0; ...
          0  0  0  0 -4  0; ...
          0  0  0  0  0 -4];
     
    % v_1 (eq. 15, solution)
    E = dot(inv(C), S_11 - dot(S_12, dot(inv(S_22), S_21)));
    
    [E_w, E_v] = eig(E);
    
    v_1 = E_v(1:end, max(E_w));
    if v_1(0) < 0
        v_1 = -v_1;
    end
    
    % v_2 (eq. 13, solution)
    v_2 = dot(dot(-inv(S_22), S_21), v_1);
    
    % quadric-form parameters
    M = [v_1(1) v_1(4) v_1(5); ...
         v_1(4) v_1(2) v_1(6); ...
         v_1(5) v_1(6) v_1(3)];
     
    n = [v_2(1); v_2(2); v_2(3)];
    
    d = v_2(4);
    
    return;
end

function cal_mag_data = calibrate(mag_data, M, n, d)
    %{
    Returns a calibrated set of magnetometer samples.

    `mag_data` should be passed as a MATLAB table with columns `[MagX, MagY, MagZ]`.
    %}
    % Note: mag_data parameter is assumed to be a MATLAB table,
    % not an array. Return type is a MATLAB table.
    
    % calculate ellipsoid parameters if not passed in
    if ~(all(M) && all(n) && all(d))
        [M, n, d] = ellipsoid_fit(table2array(mag_data).T);
    end
    
    disp(M, n, d);
    
    % calculate calibration parameters for:
    % h_m = A @ h + b where h = A^-1 @ (h_m - b)
    M_1 = inv(M);
    b = dot(M_1, n);
    A_1 = real(1 / sqrt(dot(n.T, dot(M_1, n)) - d) * sqrtm(M));
    
    % apply calibration to mag samples and return calibrated data.
    % Concern: Not sure if this is equivalent to pandas.apply().
    % original Python code: mag_data.apply(__calibrate_row, args=(A_1, b), axis=1, result_type='expand')
    cal_mag_data = varfun(private_calibrate_row, mag_data, A_1, b);

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