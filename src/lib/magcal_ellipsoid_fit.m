
% function t = transpose_table(input_table)
%     % A function used to transpose a MATLAB table.
%     % Potential alternative: rows2vars() (may not work as intended)
%     arr = table2array(input_table);
%     t = array2table(arr.');
%     
%     return;
% end

function [M, n, d] = magcal_ellipsoid_fit(s)
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
