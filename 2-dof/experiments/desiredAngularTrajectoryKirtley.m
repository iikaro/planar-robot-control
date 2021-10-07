function [q_d, dq_d] = desiredAngularTrajectoryKirtley(fileName, offset, t, dt)
dof = length(offset);
q_d = zeros(dof, length(t));
dq_d = zeros(dof, length(t));

load(fileName, 'qDesired')
sample = 2;

for i = 1 : length(t)
    if rem(i,0.005/dt) == 0
        if sample < 461
            q_d(1,i) = qDesired(sample,1);
            dq_d(1,i) = 0;
            q_d(2,i) = qDesired(sample,2);
            dq_d(2,i) = 0;
            sample = sample + 1;
        else
            sample = 2;
            q_d(1,i) = qDesired(sample,1);
            dq_d(1,i) = 0;
            q_d(2,i) = qDesired(sample,2);
            dq_d(2,i) = 0;
            sample = sample + 1;
        end
        
    else
        if i > 1
            q_d(1,i) = q_d(1,i-1);
            q_d(2,i) = q_d(2,i-1) ;
        end
    end
end

q_d(:,1:50) = 0;
q_d(1,:) = q_d(1,:) + offset(1,:);

end