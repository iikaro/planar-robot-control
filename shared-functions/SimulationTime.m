function [t, dt] = SimulationTime(t_initial, t_final, samples)
% SIMULATIONTIME  The simulation time array (t) and the time step (dt).
%
%   SIMULATIONTIME(t_initial, t_final, total_samples) receives the total
%   samples as last parameters, i.e. the length - 1 of the time array.
%
%   SIMULATIONTIME(t_initial, t_final, dt) receives the time step between
%   two consecutive elements of the t array.
%
%   If samples is greater than unit, it is considered as the total number
%   of samples. If it is less than unit, it is considered as the time step
%   itself.

if samples < 1
    dt = samples;
    t = linspace(t_initial, t_final, t_final*1/dt + 1);
else
    dt = 1/samples;
    t = linspace(t_initial, t_final, samples + 1);
end