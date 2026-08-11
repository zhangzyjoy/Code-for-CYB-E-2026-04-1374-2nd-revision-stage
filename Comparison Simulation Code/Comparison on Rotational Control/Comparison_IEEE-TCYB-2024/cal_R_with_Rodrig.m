function R = cal_R_with_Rodrig(sg)
%%%% 由罗德里格常量计算姿态旋转矩阵
R = eye(3) + 4 * ( 1 - ( ( ( sg(:)' ) * sg(:) ) / ( ( 1 + ( sg(:)' ) * sg(:) ) ^ 2 ) ) ) * cal_askew_mat( sg(:) ) ...
                            + ( 8 / ( ( 1 + ( sg(:)' ) * sg(:) ) ^ 2 ) ) * cal_askew_mat( sg(:) ) * cal_askew_mat( sg(:) );
R = R';
end
