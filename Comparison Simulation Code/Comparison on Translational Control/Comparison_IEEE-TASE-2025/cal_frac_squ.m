function y = cal_frac_squ(x, a, b)
% Compute sign(x^a) * |x^a|^(1/b)
% Logarithmic transformation is employed to prevent numerical overflow

if a < 0 && abs(norm(x, 2)) <= 1e-03
    y = zeros(size(x));
else
    % Take the absolute value of the input to avoid fractional power
    % computation issues for negative values
    abs_x = abs(x);
    
    % Preserve the sign information of the input
    sign_x = sign(x);
    
    % Compute |x|^a using logarithmic transformation
    % Special treatment is required for x = 0
    
    zero_mask = abs_x == 0;
    non_zero_mask = ~zero_mask;
    
    % Initialize the output matrix
    y = zeros(size(x));
    
    % Apply logarithmic transformation for non-zero elements
    if any(non_zero_mask(:))
        log_abs_x = log(abs_x(non_zero_mask));
        
        % Compute a * log(|x|), followed by exponential transformation
        exponent_a = a * log_abs_x;
        
        % Check whether numerical overflow may occur
        max_exp = log(realmax('double'));
        if any(exponent_a > max_exp)
            
            % Apply piecewise computation when overflow may occur
            safe_mask = exponent_a <= max_exp;
            overflow_mask = exponent_a > max_exp;
            
            % Normal computation for numerically safe elements
            if any(safe_mask)
                temp_result = exp(exponent_a(safe_mask));
                
                % Compute the (1/b)-th power
                y_temp = sign(x(non_zero_mask(safe_mask))) .^ a .* ...
                         abs(temp_result) .^ (1/b);
                
                y(non_zero_mask(safe_mask)) = y_temp;
            end
            
            % Compute overflow-prone elements directly in logarithmic form
            if any(overflow_mask)
                
                % The logarithmic form of the final expression:
                % sign(x^a) = sign(x)^a
                % log(|x^a|^(1/b)) = (a/b)*log(|x|)
                
                log_result = (a/b) * log_abs_x(overflow_mask);
                
                % Recover the final numerical value
                y_temp = sign(x(non_zero_mask(overflow_mask))) .^ a .* ...
                         exp(log_result);
                
                y(non_zero_mask(overflow_mask)) = y_temp;
            end
            
        else
            % Direct computation when no overflow risk exists
            temp_result = exp(exponent_a);
            
            y_temp = sign(x(non_zero_mask)) .^ a .* ...
                     abs(temp_result) .^ (1/b);
            
            y(non_zero_mask) = y_temp;
        end
    end
    
    % Special treatment for zero-valued elements
    if any(zero_mask(:))
        if a > 0
            y(zero_mask) = 0;
        elseif a == 0
            y(zero_mask) = 1;
        else
            y(zero_mask) = Inf;
        end
    end
end

end
