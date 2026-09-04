%% task3_train_ml_model.m
% Trains a regression model mapping scenario descriptors [z0, zd] to
% "optimal" altitude PID gains [Kp, Ki, Kd], using the dataset produced by
% task3_generate_data.m.
%
% Model choice: one shallow feed-forward net per gain (fitrnet), because
% the input space is only 2-D (z0, zd) -- a deeper/shared architecture
% would overfit. fitrnet is part of Statistics and Machine Learning
% Toolbox (R2021a+). If unavailable, an ensemble-tree fallback
% (TreeBagger) is used automatically.

clear; clc;
T = readtable('altitude_pid_dataset.csv');

X = [T.z0, T.zd];
Y = [T.Kp, T.Ki, T.Kd];
gainNames = {'Kp','Ki','Kd'};

rng(42); % reproducibility
n = height(T);
cvIdx = cvpartition(n, 'HoldOut', 0.2);
Xtr = X(training(cvIdx),:); Ytr = Y(training(cvIdx),:);
Xte = X(test(cvIdx),:);     Yte = Y(test(cvIdx),:);

models = struct();
haveFitrnet = ~isempty(which('fitrnet'));

for i = 1:3
    yName = gainNames{i};
    ytr = Ytr(:,i); yte = Yte(:,i);

    if haveFitrnet
        mdl = fitrnet(Xtr, ytr, ...
            'LayerSizes', [10 6], ...
            'Activations', 'relu', ...
            'Standardize', true, ...
            'Lambda', 1e-3);
    else
        warning('fitrnet not found -- falling back to TreeBagger ensemble for %s', yName);
        mdl = TreeBagger(50, Xtr, ytr, 'Method','regression');
    end

    if haveFitrnet
        ypred = predict(mdl, Xte);
    else
        ypred = predict(mdl, Xte);
    end

    rmse = sqrt(mean((ypred - yte).^2));
    ss_res = sum((yte - ypred).^2);
    ss_tot = sum((yte - mean(yte)).^2);
    r2 = 1 - ss_res/ss_tot;

    fprintf('%s: RMSE=%.4f  R^2=%.4f  (n_test=%d)\n', yName, rmse, r2, numel(yte));

    models.(yName) = mdl;
    models.([yName '_rmse']) = rmse;
    models.([yName '_r2']) = r2;
end

models.usesFitrnet = haveFitrnet;
models.inputRange = [min(X); max(X)]; % for extrapolation warnings later
save('pid_tuner_net.mat', 'models');
fprintf('\nSaved trained model(s) to pid_tuner_net.mat\n');

%% Quick sanity check: predict at the validated baseline point (z0=0, zd=2)
qX = [0, 2];
KpQ = predict(models.Kp, qX);
KiQ = predict(models.Ki, qX);
KdQ = predict(models.Kd, qX);
fprintf('\nSanity check at (z0=0, zd=2) [baseline scenario]:\n');
fprintf('  ML predicts Kp=%.3f Ki=%.3f Kd=%.3f\n', KpQ, KiQ, KdQ);
fprintf('  Validated baseline was Kp=56.8136 Ki=7.3365 Kd=31.7435\n');
fprintf('  (Should be close, since this point sits inside the training grid.)\n');
