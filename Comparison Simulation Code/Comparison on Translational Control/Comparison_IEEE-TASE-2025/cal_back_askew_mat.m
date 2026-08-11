function a = cal_back_askew_mat(ax)
%%%% Given the skew-symmetric matrix (a¡Á), compute the original column vector a.
a = zeros(3,1);

a(1) = ax(3,2);
a(2) = ax(1,3);
a(3) = ax(2,1);

end

