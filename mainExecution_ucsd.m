%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Description
%
%       This file uses experimental data from the Franka manipulator to 
%       learn a LDS with inputs using different data-driven methods. In the 
%       end, it saves LQR gains for all data-driven models. 
%       The data-driven learning methods used are:
%           1. Least-squares (LS) unconstrained (possibly unstable) A and B 
%              matrix pair
%           2. A learned matrix pair [A, B] with SUB, that simultaneously 
%              learns a stable A, and a B matrix. 
%           3. A learned matirx pair [A, B] with WLS, that learns a stable 
%              A, without updating the least-squares B matrix solution. 
%           4. A learned matrix pair [A,B] with CG, that learns a stable A, 
%              without updating the least-squares B matrix solution.
%
%
%
%       Given experimental data from the Franka manipulator, the code:
%           1. Combines all data (discontinuous) runs into one file
%           2. Computes the least-squares solution
%           3. Computes the SUB, WLS, and CG stable solutions
%           4. Calculates LQR gains for each method
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; close all; clc; system = 'Franka';

path_to_training_data = 'MemoryEfficientStableLDS-master/data/';
algorithms_path = '../../algorithms/'; % path for stable LDS algorithms
save_directory = 'results_svd/ucsd/';

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


dataRoot = 'prepare_data/ucsd/ucsd_seq/';
maximum = 0;
minimum = 100;
for seq = 0:253
    % disp(['current seq ', num2str(seq)]);
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
        % fprintf('Computing stable [A, B] solution using SUB ... \n');
        % 
        % timeSUB = tic;
        % [A_SUB, ~] = learnSOCmodel(X_,Y_, options);
        % tEnd = toc(timeSUB);
        % 
        % SOC_roll = rollout(A_SUB, X_(:, 1), size(X_, 2));
        % SOC_error(seq + 1, :) = mean(abs(SOC_roll - gt));
        % save(fullfile(save_state_directory, num2str(seq), 'A_SUB.mat'), "A_SUB");
        % save(fullfile(save_state_directory, 'SOC_error.mat'), "SOC_error");



        % e_SUB = norm(Y_ - A_SUB*X_, 'fro')^2/2;
        % maxeval_SUB = max(abs(eig(A_SUB)));
        % 
        % SOC_time(seq+1, stateNum - 2) = tEnd;
        % % SOC_error(seq+1, stateNum - 2) = e_SUB;
        % 
        % if(maxeval_SUB <= 1)
        %     SOC_stability(seq+1, stateNum - 2) = 1;
        % end
        % 
        % save([save_directory, 'SOC_time.mat'], 'SOC_time');
        % save([save_directory, 'SOC_error.mat'], 'SOC_error');
        % save([save_directory, 'SOC_stability.mat'], 'SOC_stability');
        % 
        % fprintf(' SOC   Max eigenvalue is : %.4f \n', maxeval_SUB);
        % fprintf(' SOC   Reconstruction error : %.5f \n', e_SUB);    

  %       %% Compute WLS (stable) [A, B] solution
  %       fprintf('Computing stable A solution using WLS ... \n');
  % 
  %       Pnew = [X_(:,1), Y_];
  %       [U_wls,S_wls,V_wls] = svd(Pnew,0);
  % 
  %       n = stateNum;
  %       V_wls = V_wls(:,1:n);
  %       S_wls = S_wls(1:n,1:n);
  %       U_wls = U_wls(:,1:n);
  % 
  %       timeWLS = tic;
  %       [A_WLS, ~, ~, ~] = learnWLSmodel(V_wls,S_wls,1,0);
  %       tEnd = toc(timeWLS);
  % 
  %       WLS_roll = rollout(A_WLS, X_(:, 1), size(X_, 2));
  %       WLS_error(seq + 1, :) = mean(abs(WLS_roll - gt));
  %       save(fullfile(save_state_directory, num2str(seq), 'A_WLS.mat'), "A_WLS");
  %       save(fullfile(save_state_directory, 'WLS_error.mat'), "WLS_error");
  % 
  %       e_WLS = norm(S_wls*V_wls(2:end,:)' - A_WLS * S_wls*V_wls(1:end-1,:)', 'fro')^2/2;
  %       maxeval_WLS = max(abs(eig(A_WLS)));
  %       if maxeval_WLS <= 1 
  %           WLS_stability(seq+1, stateNum - 2) = 1;
  %       end
  %       % WLS_error(seq+1, stateNum - 2) = e_WLS;
  %       WLS_time(seq+1, stateNum - 2) = tEnd;
  % 
  %       save([save_directory, 'WLS_time.mat'], 'WLS_time');
  %       save([save_directory, 'WLS_error.mat'], 'WLS_error');
  %       save([save_directory, 'WLS_stability.mat'], 'WLS_stability');
  % 
  %       fprintf('    Max eigenvalue is : %.4f \n', max(abs(eig(A_WLS)) ));
  %       fprintf('    Reconstruction error : %.5f \n', e_WLS);    
  % % clearvars -except system
  % 
  %       %% Compute CG (stable) [A, B] solution
  %       fprintf('Computing stable A using CG ... \n');
  % 
  %       timeCG = tic;
  %       [A_CG, ~, ~, ~] = learnCGModel(X_, Y_, 1, 0);
  %       tEnd = toc(timeCG);
  % 
  %       CG_roll = rollout(A_CG, X_(:, 1), size(X_, 2));
  %       CG_error(seq + 1, :) = mean(abs(CG_roll - gt));
  %       save(fullfile(save_state_directory, num2str(seq), 'A_CG.mat'), "A_CG");
  %       save(fullfile(save_state_directory, 'CG_error.mat'), "CG_error");
  % 
  %       e_CG = norm(Y_ - A_CG*X_, 'fro')^2/2;
  %       maxeval_CG = max(abs(eig(A_CG)));
  %       if maxeval_WLS <= 1 
  %           CG_stability(seq+1, stateNum - 2) = 1;
  %       end
  %       % CG_error(seq+1, stateNum - 2) = e_CG;
  %       CG_time(seq+1, stateNum - 2) = tEnd;
  % 
  %       save([save_directory, 'CG_time.mat'], 'CG_time');
  %       save([save_directory, 'CG_error.mat'], 'CG_error');
  %       save([save_directory, 'CG_stability.mat'], 'CG_stability');
  % 
  %       fprintf('    Max eigenvalue is : %.4f \n', maxeval_CG)
  %       fprintf('    Reconstruction error : %.5f \n', e_CG);    

    end
end
