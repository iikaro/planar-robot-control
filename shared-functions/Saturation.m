function value = Saturation(current, maximum, minimum)
% Saturation  Return the saturated value between the maximum and minimum.
%
%   value = Saturation(current, maximum, minimum)
value = min(maximum, max(minimum, current));
end