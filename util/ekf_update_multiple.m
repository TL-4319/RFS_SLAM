function [qz_update,m_update,P_update, P_d_vec] = ekf_update_multiple(z,particle,m,P, sensor_params)

plength= size(m,2);
zlength= size(z,2);

qz_update= zeros(plength,zlength);
m_update = zeros(size(m,1),plength,zlength);
P_update = zeros(size(m,1),size(m,1),plength);
P_d_vec = zeros(1, plength);

for idxp=1:plength
        [qz_temp,m_temp,P_temp, PD] = ekf_update_single(z,particle,m(:,idxp),P(:,:,idxp), sensor_params);
       qz_update(idxp,:)   = qz_temp * PD;
        m_update(:,idxp,:) = m_temp;
        P_update(:,:,idxp) = P_temp;
        P_d_vec (:,idxp) = PD;
end

function [qz_temp,m_temp,P_temp, PD] = ekf_update_single(z,particle,m,P, sensor_params)
m = vertcat(m,zeros(1,size(m,2)));
[~,eta,~, PD_mult] = gen_meas_cartesian_2D(particle.pos, particle.quat, m, sensor_params);
PD = PD_mult * sensor_params.detect_prob;
eta(3,:) = [];
m(3,:) = [];

[H_ekf,U_ekf]= ekf_update_mat(particle.quat,m);                 % user specified function for application
S= U_ekf*sensor_params.R*U_ekf'+H_ekf*P*H_ekf'; S= (S+ S')/2;   % addition step to avoid numerical problem
Vs= chol(S); det_S= prod(diag(Vs))^2; inv_sqrt_S= inv(Vs); iS= inv_sqrt_S*inv_sqrt_S';

K  = P*H_ekf'*iS;

qz_temp = exp(-0.5*size(z,1)*log(2*pi) - 0.5*log(det_S) - 0.5*dot(z-repmat(eta,[1 size(z,2)]),iS*(z-repmat(eta,[1 size(z,2)]))))';
m_temp = repmat(m,[1 size(z,2)]) + K*(z-repmat(eta,[1 size(z,2)]));
%Joeseph form
temp = (eye(2) - K * H_ekf);
P_temp = temp * P * temp' + K * sensor_params.R * K';
%P_temp = (eye(size(P))-K*H_ekf)*P;
