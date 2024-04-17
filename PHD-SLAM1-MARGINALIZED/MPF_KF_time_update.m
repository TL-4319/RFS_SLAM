function [x_pred, P_pred] = MPF_KF_time_update (x_prev, P_prev, Q)
    % Kalman time update step
    % Implement 5.2c pg 110 in Blesser's thesis
    x_pred = x_prev;
    P_pred = P_prev + Q;
end