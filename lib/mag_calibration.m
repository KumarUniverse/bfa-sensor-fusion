% sqrtm - matrix square root

function [M, n, d] = ellipsoid_fit(s)
    %{
    Calculates ellipsoid parameters to normalize magnetometer data.
    
    From: https://teslabs.com/articles/magnetometer-calibration/
    %}
    M = 0;
    n = 0;
    d = 0;
    
    return;
end

function cal_mag_data = calibrate(mag_data, M, n, d)
    %{
    Returns a calibrated set of magnetometer samples.

    `mag_data` should be passed as a subset DataFrame with columns `[MagX, MagY, MagZ]`.
    %}
    return;
end

function res = private_calibrate_row(row, A_1, b)
    %{
    Internal method to calculate correct magnetometer reading at a specific time.

    Assumes that `row` = `[MagX, MagY, MagZ]`.
    Uses correction equation `h = A^-1 @ (h_m - b)`. 
    %}
    res = [];
    return;
end
