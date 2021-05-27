ACC_COLS = {'AccelX', 'AccelY', 'AccelZ'};
GYRO_COLS = {'GyroX', 'GyroY', 'GyroZ'};
MAG_COLS = {'MagX', 'MagY', 'MagZ'};
AXES = [ACC_COLS ; GYRO_COLS ; MAG_COLS];

GRAVITY = 9.80665;

RAD_TO_DEG = 180 / pi;
%disp("RAD_TO_DEG " + RAD_TO_DEG)
DEG_TO_RAD = pi / 180;
%disp("DEG_TO_RAD " + DEG_TO_RAD)