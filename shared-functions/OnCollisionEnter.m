function [value] = OnCollisionEnter(x, x_w)
value = 0;
if x_w(1) > 0;
    if x(1) > x_w(1), value = 1; end
    return
elseif x_w(1) < 0;
    if x(2) < x_w(2), value = 1; end
    return
end
end