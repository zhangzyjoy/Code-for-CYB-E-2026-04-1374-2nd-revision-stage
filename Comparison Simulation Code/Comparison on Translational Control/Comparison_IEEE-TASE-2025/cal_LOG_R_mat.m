function logR = cal_LOG_R_mat(R, order)
%%%% The input variable <R> denotes the attitude rotation matrix
%%%% belonging to the SO(3) manifold, where R^T = R^(-1)
%%%% The input variable <order> specifies the truncation order of the
%%%% Taylor series expansion for approximating the logarithmic map

logR = zeros(3,3);
for i = 1:1:order
    logR = logR + ((-1)^(i+1)) * ((R - eye(3,3))^i) / i;
end

end

