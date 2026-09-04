% Generates the training dataset for ML-based self-tuning of the altitude PID in Quadcopter_Dynamics_ver2.slx.

% For each scenario (z0, zd) in a DOE grid:
%   1) set Integrator IC(5) = z0, Step.FinalValue = zd
%   2) run baseline sim to get a starting cost
%   3) locally optimize (Kp,Ki,Kd) around the baseline gains to minimize
%      ITAE + overshoot penalty (fminsearch, bounded via penalty terms)
%   4) log [z0, zd, Kp*, Ki*, Kd*, metrics] to a table / CSV

clear; clc;
modelName = 'Quadcopter_Dynamics_ver2';
load_system(modelName);

pidPath = [modelName '/PID Controller'];
stepPath = [modelName '/Step'];
intPath  = [modelName '/Integrator'];

%%baseline
baseline.Kp = str2double(get_param(pidPath,'P'));
baseline.Ki = str2double(get_param(pidPath,'I'));
baseline.Kd = str2double(get_param(pidPath,'D'));
baseline.zd = str2double(get_param(stepPath,'FinalValue'));
baseline.IC = get_param(intPath,'InitialCondition'); % 'zeros(12,1)'
fprintf('Baseline gains: Kp=%.4f Ki=%.4f Kd=%.4f\n', baseline.Kp, baseline.Ki, baseline.Kd);

cleanupObj = onCleanup(@() restoreBaseline(pidPath, stepPath, intPath, baseline));

%% DOE GRIDE
z0_list = [0, 0.25, 0.5, 1.0];          % initial altitude [m]
zd_list = [0.5, 1.0, 1.5, 2.0, 3.0, 4.0]; % commanded altitude [m] (2.0 = validated baseline case)

Tstop = 8;      % [s] simulation horizon per scenario (altitude loop settles well within this)
gainLB = 0.4;   % lower bound multiplier on baseline gains for the local search
gainUB = 1.8;   % upper bound multiplier on baseline gains for the local search

rows = {};
rowIdx = 0;

opts = optimset('Display','off','MaxFunEvals',150,'MaxIter',150,'TolX',1e-3,'TolFun',1e-3);

for z0 = z0_list
    for zd = zd_list
        if abs(z0 - zd) < 1e-6
            continue 
        end
        fprintf('--- Scenario z0=%.2f  zd=%.2f ---\n', z0, zd);

        % Configure scenario on the model
        setScenario(intPath, stepPath, z0, zd);

        % Cost function: ITAE + overshoot penalty, gains passed as ratios of baseline so the search space is well-scaled.
        costFun = @(g) simCost(modelName, pidPath, g, baseline, gainLB, gainUB, Tstop, zd);

        g0 = [1 1 1]; % start at baseline ratios
        [gStar, costStar] = fminsearch(costFun, g0, opts);
        gStar = clampRatio(gStar, gainLB, gainUB);

        KpStar = gStar(1)*baseline.Kp;
        KiStar = gStar(2)*baseline.Ki;
        KdStar = gStar(3)*baseline.Kd;

        % Re-simulate at the found optimum to log clean metrics
        set_param(pidPath,'P',num2str(KpStar),'I',num2str(KiStar),'D',num2str(KdStar));
        m = simulateAndMeasure(modelName, Tstop, zd);

        rowIdx = rowIdx + 1;
        rows(rowIdx,:) = {z0, zd, KpStar, KiStar, KdStar, m.overshoot_pct, ...
                           m.rise_time, m.settling_time, m.itae, costStar};

        fprintf('   -> Kp=%.3f Ki=%.3f Kd=%.3f | OS=%.1f%% Tr=%.2fs Ts=%.2fs ITAE=%.3f\n', ...
            KpStar, KiStar, KdStar, m.overshoot_pct, m.rise_time, m.settling_time, m.itae);
    end
end

T = cell2table(rows, 'VariableNames', ...
    {'z0','zd','Kp','Ki','Kd','overshoot_pct','rise_time','settling_time','itae','cost'});
writetable(T, 'altitude_pid_dataset.csv');
fprintf('\nSaved %d scenarios to altitude_pid_dataset.csv\n', height(T));

%%
function setScenario(intPath, stepPath, z0, zd)
    ic = zeros(12,1); ic(5) = z0;
    set_param(intPath,'InitialCondition', mat2str(ic));
    set_param(stepPath,'FinalValue', num2str(zd));
end

function r = clampRatio(g, lb, ub)
    r = min(max(g, lb), ub);
end

function J = simCost(modelName, pidPath, g, baseline, lb, ub, Tstop, zd)
    gC = clampRatio(g, lb, ub);
    penalty = 50*sum((g - gC).^2); % soft penalty for going outside bounds

    Kp = gC(1)*baseline.Kp; Ki = gC(2)*baseline.Ki; Kd = gC(3)*baseline.Kd;
    set_param(pidPath,'P',num2str(Kp),'I',num2str(Ki),'D',num2str(Kd));

    m = simulateAndMeasure(modelName, Tstop, zd);

    % Cost: ITAE (tracking speed/accuracy) + overshoot penalty (>2% starts costing)
    osPenalty = max(0, m.overshoot_pct - 2)^2;
    J = m.itae + 0.5*osPenalty + penalty;
    if ~isfinite(J), J = 1e6; end
end

function m = simulateAndMeasure(modelName, Tstop, zd)
    simOut = sim(modelName, 'StopTime', num2str(Tstop));
    % Pull altitude (Selector output feeds Scope) via logged signal.
    % Requires signal logging enabled on the Selector->Sum wire (see note below).
    logsout = simOut.get('logsout');
    if isempty(logsout) || logsout.numElements == 0
        error(['No logged signal found. In ver2.slx, right-click the ' ...
               'Selector output wire -> "Log Selected Signal" (name it "z") ' ...
               'once, then re-save the model before running this script.']);
    end
    zSig = logsout.getElement('z');
    t = zSig.Values.Time;
    z = zSig.Values.Data;

    e = zd - z;
    m.itae = trapz(t, t(:).*abs(e(:)));

    if zd > 0
        m.overshoot_pct = max(0, (max(z) - zd)/zd*100);
    else
        m.overshoot_pct = max(0, max(z) - zd);
    end

    % Rise time: 10%-90% of the step
    z0v = z(1);
    lo = z0v + 0.1*(zd - z0v); hi = z0v + 0.9*(zd - z0v);
    try
        tlo = t(find(sign(zd-z0v)*(z-lo) >= 0, 1, 'first'));
        thi = t(find(sign(zd-z0v)*(z-hi) >= 0, 1, 'first'));
        m.rise_time = thi - tlo;
    catch
        m.rise_time = NaN;
    end

    % Settling time: last time |e| exceeds 2% band
    band = 0.02*max(abs(zd), 1e-6);
    idxOut = find(abs(e) > band, 1, 'last');
    if isempty(idxOut)
        m.settling_time = 0;
    else
        m.settling_time = t(min(idxOut+1, numel(t)));
    end
end

function restoreBaseline(pidPath, stepPath, intPath, baseline)
    set_param(pidPath,'P',num2str(baseline.Kp),'I',num2str(baseline.Ki),'D',num2str(baseline.Kd));
    set_param(stepPath,'FinalValue', num2str(baseline.zd));
    set_param(intPath,'InitialCondition', baseline.IC);
    fprintf('Baseline model parameters restored.\n');
end
