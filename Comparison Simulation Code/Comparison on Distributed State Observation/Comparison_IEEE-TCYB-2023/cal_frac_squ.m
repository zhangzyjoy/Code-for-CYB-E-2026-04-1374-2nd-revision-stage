function y = cal_frac_squ(x, a, b)
y = sign(x .^ a) .* abs( (x .^ a) .^ (1/b));
end
