function ax = cal_askew_mat(a)
%%%% Compute the skew-symmetric matrix associated with the column vector a
a = a(:);
ax = [0,      -a(3),      a(2); ...
         a(3),     0,        -a(1);...
        -a(2),   a(1),         0   ];

end

