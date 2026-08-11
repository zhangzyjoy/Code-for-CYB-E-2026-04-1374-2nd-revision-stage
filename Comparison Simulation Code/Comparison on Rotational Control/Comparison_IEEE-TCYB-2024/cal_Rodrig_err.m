function sge = cal_Rodrig_err(sg, sgc)
%%%% 根据罗德里格常量 sg 和罗德里格常量指令 sgc 计算 罗德里格常量误差 sge
sge =  ( 1 / ( 1 + ( ( sgc(:)' ) * sgc(:) ) * ( ( sg(:)' ) * sg(:) ) + 2 * ( sgc(:)' ) * sg(:) ) ) * ...
            ( ( ( sg(:)' ) * sg(:) - 1 ) * sgc(:) + ( 1 - ( sgc(:)' ) * sgc(:) ) * sg(:) - 2 * cal_askew_mat( sgc(:) ) * sg(:) );
end
