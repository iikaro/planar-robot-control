function [q_p, dq_p] = patientJoints(A, w, offset, t)
dof = length(offset);

q_p = zeros(dof, length(t));
dq_p = zeros(dof, length(t));

% First joint
q_p(1,:) = A*sin(w*t) + offset(1) + pi/6;
dq_p(1,:) = w*A*cos(w*t);

% Second joint
q_p(2,:) = A*sin(w*t) + offset(2);
dq_p(2,:) = w*A*cos(w*t);
end