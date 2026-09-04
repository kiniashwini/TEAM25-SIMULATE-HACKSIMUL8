% Implements the ML self-tuning altitude PID and compares it against the fixed-gain baseline on unseen (z0, zd) test cases.

% Implementation "mission-start" self-tuning. Before each
%       run, z0 and zd are known (initial altitude + commanded altitude),
%       so the ML model predicts gains ONCE, and the PID Controller
%       block's P/I/D parameters are set via set_param before simulating.
%       This is a faithful, easily-verified implementation of ML-based
%       gain self-tuning for this loop, and requires no structural
%       changes to the validated block diagram.

clear; clc;
modelName = 'Quadcopter_Dynamics_ver2';
load_system(modelName);
load('pid_tuner_net.mat'); % -> models

pidPath  = [modelName '/PID Controller'];
stepPath = [modelName '/Step'];
intPath  = [modelName '/Integrator'];

baseline.Kp = 56.8136; baseline.Ki = 7.3365; baseline.Kd = 31.7435;
baseline.zd = str2double(get_param(stepPath,'FinalValue'));
baseline.IC = get_param(intPath,'InitialCondition');
cleanupObj = onCleanup(@() restoreBaseline(pidPath, stepPath, intPath, baseline));

Tstop = 8;

% Test cases
testCases = [
    0.10, 1.75;   % near-baseline, interpolated
    0.75, 3.50;   
    0.00, 2.00;   % exact validated baseline case (sanity anchor)
   -0.00, 4.75;   % mild extrapolation
];

results = table();

for i = 1:size(testCases,1)
    z0 = testCases(i,1); zd = testCases(i,2);
    fprintf('\n=== Test case %d: z0=%.2f  zd=%.2f ===\n', i, z0, zd);

    % Warn if extrapolating outside the training range
    inRange = all([z0 zd] >= models.inputRange(1,:)) && all([z0 zd] <= models.inputRange(2,:));
    if ~inRange
        fprintf('  [note] (z0,zd) is outside the training range -- extrapolation, treat with caution.\n');
    end

    % Baseline (fixed gains) run
    setScenario(intPath, stepPath, z0, zd);
    set_param(pidPath,'P',num2str(baseline.Kp),'I',num2str(baseline.Ki),'D',num2str(baseline.Kd));
    [tB, zB] = runAndLog(modelName, Tstop);
    mB = metricsFrom(tB, zB, zd);

    % ML self-tuned run
    Kp = predict(models.Kp, [z0 zd]);
    Ki = predict(models.Ki, [z0 zd]);
    Kd = predict(models.Kd, [z0 zd]);
    set_param(pidPath,'P',num2str(Kp),'I',num2str(Ki),'D',num2str(Kd));
    [tM, zM] = runAndLog(modelName, Tstop);
    mM = metricsFrom(tM, zM, zd);

    fprintf('  Baseline : Kp=%.2f Ki=%.2f Kd=%.2f | OS=%.1f%% Tr=%.2fs Ts=%.2fs ITAE=%.3f\n', ...
        baseline.Kp, baseline.Ki, baseline.Kd, mB.overshoot_pct, mB.rise_time, mB.settling_time, mB.itae);
    fprintf('  ML-tuned : Kp=%.2f Ki=%.2f Kd=%.2f | OS=%.1f%% Tr=%.2fs Ts=%.2fs ITAE=%.3f\n', ...
        Kp, Ki, Kd, mM.overshoot_pct, mM.rise_time, mM.settling_time, mM.itae);

    results = [results; { i, z0, zd, ...
        baseline.Kp, baseline.Ki, baseline.Kd, mB.overshoot_pct, mB.rise_time, mB.settling_time, mB.itae, ...
        Kp, Ki, Kd, mM.overshoot_pct, mM.rise_time, mM.settling_time, mM.itae }];

    % Overlay plot
    figure('Name', sprintf('Test case %d: z0=%.2f zd=%.2f', i, z0, zd));
    plot(tB, zB, 'LineWidth', 1.5); hold on;
    plot(tM, zM, 'LineWidth', 1.5);
    yline(zd, '--k');
    legend('Baseline fixed-gain PID','ML self-tuned PID','Reference z_d','Location','best');
    xlabel('Time [s]'); ylabel('Altitude z [m]');
    title(sprintf('Altitude response, z_0=%.2f m \\rightarrow z_d=%.2f m', z0, zd));
    grid on;
end

results.Properties.VariableNames = {'case','z0','zd', ...
    'Kp_base','Ki_base','Kd_base','OS_base','Tr_base','Ts_base','ITAE_base', ...
    'Kp_ml','Ki_ml','Kd_ml','OS_ml','Tr_ml','Ts_ml','ITAE_ml'};
writetable(results, 'task4_comparison_results.csv');

fprintf('\n Summary (ITAE improvement, negative = ML better) ===\n');
for i = 1:height(results)
    dITAE = (results.ITAE_ml(i) - results.ITAE_base(i))/results.ITAE_base(i)*100;
    fprintf('  Case %d: ITAE change = %+.1f%%,  overshoot: %.1f%% -> %.1f%%\n', ...
        i, dITAE, results.OS_base(i), results.OS_ml(i));
end


%% -
function setScenario(intPath, stepPath, z0, zd)
    ic = zeros(12,1); ic(5) = z0;
    set_param(intPath,'InitialCondition', mat2str(ic));
    set_param(stepPath,'FinalValue', num2str(zd));
end

function [t, z] = runAndLog(modelName, Tstop)
    simOut = sim(modelName, 'StopTime', num2str(Tstop));
    logsout = simOut.get('logsout');
    zSig = logsout.getElement('z');
    t = zSig.Values.Time;
    z = zSig.Values.Data;
end

function m = metricsFrom(t, z, zd)
    e = zd - z;
    m.itae = trapz(t, t(:).*abs(e(:)));
    z0v = z(1);
    if zd > 0
        m.overshoot_pct = max(0, (max(z) - zd)/zd*100);
    else
        m.overshoot_pct = max(0, max(z) - zd);
    end
    lo = z0v + 0.1*(zd - z0v); hi = z0v + 0.9*(zd - z0v);
    try
        tlo = t(find(sign(zd-z0v)*(z-lo) >= 0, 1, 'first'));
        thi = t(find(sign(zd-z0v)*(z-hi) >= 0, 1, 'first'));
        m.rise_time = thi - tlo;
    catch
        m.rise_time = NaN;
    end
    band = 0.02*max(abs(zd), 1e-6);
    idxOut = find(abs(e) > band, 1, 'last');
    if isempty(idxOut), m.settling_time = 0; else, m.settling_time = t(min(idxOut+1, numel(t))); end
end

function restoreBaseline(pidPath, stepPath, intPath, baseline)
    set_param(pidPath,'P',num2str(baseline.Kp),'I',num2str(baseline.Ki),'D',num2str(baseline.Kd));
    set_param(stepPath,'FinalValue', num2str(baseline.zd));
    set_param(intPath,'InitialCondition', baseline.IC);
end