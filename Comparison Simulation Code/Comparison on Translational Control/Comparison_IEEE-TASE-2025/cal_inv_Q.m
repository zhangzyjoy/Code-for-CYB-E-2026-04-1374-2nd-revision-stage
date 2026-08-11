function Qinv = cal_inv_Q(Q)
% Compute the inverse of the quaternion:
% For Q = [theta_i, q_i], the inverse is given by Q^{-1} = [theta_i, -q_i]

theta = Q(1,1);
q = Q(2:4,1);
Qinv = [theta; -q];

end

