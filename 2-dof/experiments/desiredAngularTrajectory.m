function [q_d, dq_d] = desiredAngularTrajectory(A, w, offset, t)
% First joint
q_d(1,:) = A*sin(w*t) + offset(1);
dq_d(1,:) = w*A*cos(w*t);

% Second joint
q_d(2,:) = A*sin(w*t) + offset(2);
dq_d(2,:) = w*A*cos(w*t);

for i = 1 : length(t)
    if t(i) >= 2.5, q_d(2,i) = q_d(2,i-1); q_d(1,i) = q_d(1,i-1); end
end

end