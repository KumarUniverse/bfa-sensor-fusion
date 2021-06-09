ACC_COLS = {'AccelX', 'AccelY', 'AccelZ'};
GYRO_COLS = {'GyroX', 'GyroY', 'GyroZ'};
MAG_COLS = {'MagX', 'MagY', 'MagZ'};
AXES = [ACC_COLS ; GYRO_COLS ; MAG_COLS];

GRAVITY = 9.80665;

RAD_TO_DEG = 180 / pi;
DEG_TO_RAD = pi / 180;