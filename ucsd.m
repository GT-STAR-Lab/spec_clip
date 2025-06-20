clear; close all; clc;

algorithms_path = '../../algorithms/'; % path for stable LDS algorithms
save_directory = 'results_ucsd/';

addpath(algorithms_path);
options.graphic = 0;
options.posdef = 10e-12;
options.maxiter = 200000;

% settings;
nStates = 17; % Number of system states
nControl = 7; % Number of system inputs


LS_error = zeros(254, 27);
SC_error = zeros(254, 27);
WLS_error = zeros(254, 27);
CG_error = zeros(254, 27);
SOC_error = zeros(254, 27);

LS_time = zeros(254, 27);
SC_time = zeros(254, 27);
WLS_time = zeros(254, 27);
CG_time = zeros(254, 27);
SOC_time = zeros(254, 27);

LS_stability = zeros(254, 27);
SC_stability = zeros(254, 27);
WLS_stability = zeros(254, 27);
CG_stability = zeros(254, 27);
SOC_stability = zeros(254, 27);


dataRoot = 'datasets/ucsd/ucsd_seq/';
maximum = 0;
minimum = 100;
for seq = 0:253
    
    loc = [dataRoot, 'ucsdseq_', num2str(seq), '.mat'];
    data = load(loc).data;
    [~, l] = size(data);
    if l > maximum
        maximum = l;
    end
    if l < minimum
        minimum = l;
    end

end
for stateNum = [3:30, 3]
    SC_error = zeros(254, 42);
    SC99_error = zeros(254, 42);
    SC99999_error = zeros(254, 42);
    LS_error = zeros(254, 42);
    SOC_error = zeros(254, 42);
    WLS_error = zeros(254, 42);
    CG_error = zeros(254, 42);

    for seq = 0:253
        disp(['current seq ', num2str(seq)]);
        loc = [dataRoot, 'ucsdseq_', num2str(seq), '.mat'];
        data = load(loc).data(:, 1:42);
        % Y = data(2:end, :);
        % X = data(1:end - 1, :);
        [data_U, data_S, data_V] = svd(data);


    
        subspace_S = data_S(1:stateNum, :);
        subspace_data = subspace_S * data_V.';
        disp(['dimension ', num2str(stateNum)])
        X_ = subspace_data(:, 1:end - 1);
        Y_ = subspace_data(:, 2:end);


        % %% Compute LS (unconstrained) and SC [A, B] solution
        fprintf('Computing LS unconstrained and SC [A, B] solution ... \n');
        randomShuffling = false;
        tStart = tic;
        tStart_SC = tic;
        LS = LDS(X_, Y_);
        tLS = toc(tStart);
        SC = eigenclip(LS, 1);
        SC99 = eigenclip(LS, 0.99);
        SC99999 = eigenclip(LS, 0.99999);
        tSC = toc(tStart_SC);

        SC_roll = rollout(SC, X_(:, 1), size(X_, 2));
        LS_roll = rollout(LS, X_(:, 1), size(X_, 2));
        SC99_roll = rollout(SC99, X_(:, 1), size(X_, 2)); 
        SC99999_roll = rollout(SC99999, X_(:, 1), size(X_, 2));

        gt = [X_, Y_(:, end)];
        
        SC_error(seq + 1, :) = mean(abs(SC_roll - gt));
        LS_error(seq + 1, :) = mean(abs(LS_roll - gt));
        SC99_error(seq + 1, :) = mean(abs(SC99_roll - gt));
        SC99999_error(seq + 1, :) = mean(abs(SC99999_roll - gt));
        
        save_state_directory = fullfile(save_directory, num2str(stateNum));
        mkdir(save_state_directory);
        mkdir(fullfile(save_state_directory, num2str(seq)));

        save(fullfile(save_state_directory, num2str(seq), 'SC.mat'), "SC");
        save(fullfile(save_state_directory, num2str(seq), 'LS.mat'), "LS");

        save(fullfile(save_state_directory, 'SC_error.mat'), "SC_error");
        save(fullfile(save_state_directory, 'LS_error.mat'), "LS_error");

        save(fullfile(save_state_directory, num2str(seq), 'SC99.mat'), "SC99");
        save(fullfile(save_state_directory, 'SC99_error.mat'), "SC99_error");

        save(fullfile(save_state_directory, num2str(seq), 'SC99999.mat'), "SC99999");
        save(fullfile(save_state_directory, 'SC99999_error.mat'), "SC99999_error");


        LS_time(seq+1, stateNum - 2) = tLS;
        SC_time(seq+1, stateNum - 2) = tSC;

        save([save_directory, 'LS_time.mat'], 'LS_time');
        save([save_directory, 'SC_time.mat'], 'SC_time');




        %% Compute SUB (stable) [A, B] solution
        fprintf('Computing stable [A, B] solution using SUB ... \n');

        timeSOC = tic;
        [SOC, ~] = learnSOCmodel(X_,Y_, options);
        tSOC = toc(timeSOC);

        SOC_roll = rollout(SOC, X_(:, 1), size(X_, 2));
        SOC_error(seq + 1, :) = mean(abs(SOC_roll - gt));
        save(fullfile(save_state_directory, num2str(seq), 'SOC.mat'), "SOC");
        save(fullfile(save_state_directory, 'SOC_error.mat'), "SOC_error");

        SOC_time(seq+1, stateNum - 2) = tSOC;

        save([save_directory, 'SOC_time.mat'], 'SOC_time');

        Pnew = [X_(:,1), Y_];
        [U_wls,S_wls,V_wls] = svd(Pnew,0);

        n = stateNum;
        V_wls = V_wls(:,1:n);
        S_wls = S_wls(1:n,1:n);
        U_wls = U_wls(:,1:n);

        timeWLS = tic;
        [WLS, ~, ~, ~] = learnWLSmodel(V_wls,S_wls,1,0);
        tWLS = toc(timeWLS);

        WLS_roll = rollout(WLS, X_(:, 1), size(X_, 2));
        WLS_error(seq + 1, :) = mean(abs(WLS_roll - gt));
        save(fullfile(save_state_directory, num2str(seq), 'WLS.mat'), "WLS");
        save(fullfile(save_state_directory, 'WLS_error.mat'), "WLS_error");
        WLS_time(seq+1, stateNum - 2) = tWLS;
        save([save_directory, 'WLS_time.mat'], 'WLS_time');

        timeCG = tic;
        [CG, ~, ~, ~] = learnCGModel(X_, Y_, 1, 0);
        tCG = toc(timeCG);

        CG_roll = rollout(CG, X_(:, 1), size(X_, 2));
        CG_error(seq + 1, :) = mean(abs(CG_roll - gt));
        save(fullfile(save_state_directory, num2str(seq), 'CG.mat'), "CG");
        save(fullfile(save_state_directory, 'CG_error.mat'), "CG_error");

        CG_time(seq+1, stateNum - 2) = tEnd;
        
        save([save_directory, 'CG_time.mat'], 'CG_time');
        save([save_directory, 'CG_error.mat'], 'CG_error');
        save([save_directory, 'CG_stability.mat'], 'CG_stability');

    end
end
