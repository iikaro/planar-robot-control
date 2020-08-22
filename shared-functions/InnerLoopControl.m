function input = InnerLoopControl(k_p, k_d, k_i, error, d_error, int_error, Jacobian)
switch nargin
    case 6
        input = k_p * error  + k_d * d_error + k_i * int_error;
    case 7
        input = Jacobian' * ( k_p * error  + k_d * d_error + k_i * int_error);
end
end