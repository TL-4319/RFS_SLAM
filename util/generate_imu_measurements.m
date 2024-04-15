function imu_meas = generate_imu_measurements(acc_world, rot_vel_world, quat, imu_param)
    %% Perfect IMU just for testing 
    IMU_perfect = imuSensor('accel-gyro','SampleRate',imu_param.dt);
    [perfect_accel, perfect_gyro] = IMU_perfect(acc_world', rot_vel_world', quat);
    perfect_imu.accel = perfect_accel';
    perfect_imu.gyro = perfect_gyro';

    %% No bias imu
    IMU_no_bias = imuSensor('accel-gyro','SampleRate',imu_param.dt);
    % Noise characteristic of IMU
    IMU_no_bias.Accelerometer = accelparams( ...
        'MeasurementRange',19.62, ...            % m/s^2
        'NoiseDensity',imu_param.accel_NoiseDensity);               % m/s^2 / Hz^(1/2)
    
    IMU_no_bias.Gyroscope = gyroparams(...
        'MeasurementRange',4.363, ...   % rad/s
        'NoiseDensity', imu_param.gyro_NoiseDensity);      % rad/s / Hz^(1/2)
    [no_bias_accel, no_bias_gyro] = IMU_no_bias(acc_world', rot_vel_world', quat);
    no_bias_imu.accel = no_bias_accel';
    no_bias_imu.gyro = no_bias_gyro';
    

    %% Normal imu
    IMU = imuSensor('accel-gyro','SampleRate',imu_param.dt);
    %Noise characteristic of IMU
    IMU.Accelerometer = accelparams( ...
        'MeasurementRange',19.62, ...            % m/s^2
        'NoiseDensity',imu_param.accel_NoiseDensity, ...      % m/s^2 / Hz^(1/2)
        'RandomWalk',imu_param.accel_RandomWalk);          % m/s^2 * Hz^(1/2)   0.086  
    
    IMU.Gyroscope = gyroparams(...
        'MeasurementRange',4.363, ...   % rad/s
        'NoiseDensity', imu_param.gyro_NoiseDensity,...     % rad/s / Hz^(1/2)
        'RandomWalk',imu_param.gyro_RandomWalk);      % rad/s * Hz^(1/2) 0.022
    [imu_accel, imu_gyro] = IMU(acc_world', rot_vel_world', quat);
    imu.accel = imu_accel';
    imu.gyro = imu_gyro';

    % Combine several simulated imu to one object
    imu_meas.perfect_imu = perfect_imu;
    imu_meas.no_bias_imu = no_bias_imu;
    imu_meas.imu = imu;
end