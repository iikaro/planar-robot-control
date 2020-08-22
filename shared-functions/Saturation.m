function value = Saturation(current, maximum, minimum)
% Saturation  Saturated value between given maximum and minimum.
%   Saturation(current, maximum, minim) returns the saturated value, given
%   the current value to be analyzed and its saturation limits.

%   value = Saturation(current, maximum, minimum)
value = min(maximum, max(minimum, current));
end