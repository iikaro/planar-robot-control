function value = Saturation(current, maximum, minimum)
% Saturation  Saturated value between given maximum and minimum.
%   Saturation(current, maximum, minim) returns the saturated value, given
%   the current value to be analyzed and its saturation limits.

%   value = Saturation(current, maximum, minimum)
for i = 1:length(current)
    value(i,:) = min(maximum(i,:), max(minimum(i,:), current(i,:)));
end
end