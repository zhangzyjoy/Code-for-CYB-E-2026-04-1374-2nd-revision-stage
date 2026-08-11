function y = cal_frac_squ(x, a, b)
if a < 0 && abs( norm(x,2) ) <= 1e-03
    y = zeros(size(x));
else
    y = sign(x .^ a) .* abs( (x .^ a) .^ (1/b));
end

end
