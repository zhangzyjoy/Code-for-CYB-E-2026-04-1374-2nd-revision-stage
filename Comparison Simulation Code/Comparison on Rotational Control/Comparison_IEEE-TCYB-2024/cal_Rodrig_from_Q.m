function sg = cal_Rodrig_from_Q(Q)
%%%% 由四元数计算罗德里格常量
sg = Q(2:4,1) ./ ( 1 + Q(1,1) );
sg = sg(:);
end
