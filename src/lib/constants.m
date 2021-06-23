ACC_COLS = {'AccelX', 'AccelY', 'AccelZ'};
GYRO_COLS = {'GyroX', 'GyroY', 'GyroZ'};
MAG_COLS = {'MagX', 'MagY', 'MagZ'};
AXES = cat(1, ACC_COLS, GYRO_COLS, MAG_COLS); %[ACC_COLS ; GYRO_COLS ; MAG_COLS];

GRAVITY = 9.80665;

RAD_TO_DEG = 180 / pi;
DEG_TO_RAD = pi / 180;

% Save all the variables to a data file.
save('constants.mat', 'ACC_COLS', 'GYRO_COLS', 'MAG_COLS', 'AXES', ...
    'GRAVITY', 'RAD_TO_DEG', 'DEG_TO_RAD');