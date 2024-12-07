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
algorithms_path = 'algorithms/'; % path for stable LDS algorithms
save_directory = 'results_svd/dtdb/';
status = 0;

addpath(algorithms_path);
options.graphic = 0;
options.posdef = 10e-12;
options.maxiter = 200000;

% settings;
nStates = 17; % Number of system states
nControl = 7; % Number of system inputs

AllDimensions = 3:30;

if(status == 0)
    unclip_error = zeros(254, 27);
    clip_error = zeros(254, 27);
    WLS_error = zeros(254, 27);
    CG_error = zeros(254, 27);
    SOC_error = zeros(254, 27);
    
    unclip_time = zeros(254, 27);
    clip_time = zeros(254, 27);
    WLS_time = zeros(254, 27);
    CG_time = zeros(254, 27);
    SOC_time = zeros(254, 27);
    
    unclip_stability = zeros(254, 27);
    clip_stability = zeros(254, 27);
    WLS_stability = zeros(254, 27);
    CG_stability = zeros(254, 27);
    SOC_stability = zeros(254, 27);
else
    CG_error = load('results_svd/dtdb/CG_error.mat').CG_error;
    CG_time = load('results_svd/dtdb/CG_time.mat').CG_time;
    CG_stability = load('results_svd/dtdb/CG_stability.mat').CG_stability;
    
    clip_error = load('results_svd/dtdb/clip_error.mat').clip_error;
    clip_time = load('results_svd/dtdb/clip_time.mat').clip_time;
    clip_stability = load('results_svd/dtdb/clip_stability.mat').clip_stability;
    
    unclip_error = load('results_svd/dtdb/unclip_error.mat').unclip_error;
    unclip_time = load('results_svd/dtdb/unclip_time.mat').unclip_time;
    unclip_stability = load('results_svd/dtdb/unclip_stability.mat').unclip_stability;
    
    WLS_error = load('results_svd/dtdb/WLS_error.mat').WLS_error;
    WLS_time = load('results_svd/dtdb/WLS_time.mat').WLS_time;
    WLS_stability = load('results_svd/dtdb/WLS_stability.mat').WLS_stability;
    
    SOC_error = load('results_svd/dtdb/SOC_error.mat').SOC_error;
    SOC_time = load('results_svd/dtdb/SOC_time.mat').SOC_time;
    SOC_stability = load('results_svd/dtdb/SOC_stability.mat').SOC_stability;
end

dataRoot = 'prepare_data\dtdb\prepared\';
for seq = 0:285
    disp(['current seq ', num2str(seq)]);
    loc = [dataRoot, 'dtdb_', num2str(seq + 1), '.mat'];
    data = double(load(loc).seq.') / 255.0;
    [total_state, length] = size(data);
    if(length > 300)
        data = data(:, 1:300);
    end
    % Y = data(2:end, :);
    % X = data(1:end - 1, :);
    [data_U, data_S, data_V] = svd(data);


    for stateNum = 3:30
        subspace_S = data_S(1:stateNum, :);
        subspace_data = subspace_S * data_V.';
        disp(['dimension ', num2str(stateNum)])
        X_ = subspace_data(:, 1:end - 1);
        Y_ = subspace_data(:, 2:end);


        % %% Compute LS (unconstrained) [A, B] solution
        % fprintf('Computing LS unconstrained [A, B] solution ... \n');
        % randomShuffling = false;
        % tStart = tic;
        % [clLS, unclLS] = LDS(X_, Y_);
        % tEnd = toc(tStart);
        % disp(tEnd);
        % cl_e_LS = norm(Y_ - clLS * X_, 'fro')^2/2;
        % uncl_e_LS = norm(Y_ - unclLS * X_, 'fro')^2/2;
        % error(stateNum - 2) = uncl_e_LS;
        % maxeval_LS = max(abs(eig(clLS)) );
        % maxeval_LS_unclipped = max(abs(eig(unclLS)) );
        % 
        % if(maxeval_LS_unclipped <= 1)
        %     unclip_stability(seq+1, stateNum - 2) = 1;
        % end
        % if(maxeval_LS <= 1)
        %     clip_stability(seq+1, stateNum - 2) = 1;
        % end
        % 
        % unclip_time(seq+1, stateNum - 2) = tEnd;
        % clip_time(seq+1, stateNum - 2) = tEnd;
        % 
        % unclip_error(seq+1, stateNum - 2) = uncl_e_LS;
        % clip_error(seq+1, stateNum - 2) = cl_e_LS;
        % 
        % save([save_directory, 'unclip_time.mat'], 'unclip_time');
        % save([save_directory, 'unclip_error.mat'], 'unclip_error');
        % save([save_directory, 'unclip_stability.mat'], 'unclip_stability');
        % 
        % save([save_directory, 'clip_time.mat'], 'clip_time');
        % save([save_directory, 'clip_error.mat'], 'clip_error');
        % save([save_directory, 'clip_stability.mat'], 'clip_stability');
        % 
        % fprintf('  LS  Max eigenvalue is : %.4f \n', maxeval_LS);
        % fprintf('  LS  Reconstruction error : %.5f \n', cl_e_LS);



        %% Compute SUB (stable) [A, B] solution
        fprintf('Computing stable [A, B] solution using SUB ... \n');

        timeSUB = tic;
        [A_SUB, ~] = learnSOCmodel(X_,Y_, options);
        tEnd = toc(timeSUB);

        e_SUB = norm(Y_ - A_SUB*X_, 'fro')^2/2;
        maxeval_SUB = max(abs(eig(A_SUB)));

        SOC_time(seq+1, stateNum - 2) = tEnd;
        SOC_error(seq+1, stateNum - 2) = e_SUB;

        if(maxeval_SUB <= 1)
            SOC_stability(seq+1, stateNum - 2) = 1;
        end

        save([save_directory, 'SOC_time.mat'], 'SOC_time');
        save([save_directory, 'SOC_error.mat'], 'SOC_error');
        save([save_directory, 'SOC_stability.mat'], 'SOC_stability');

        fprintf(' SOC   Max eigenvalue is : %.4f \n', maxeval_SUB);
        fprintf(' SOC   Reconstruction error : %.5f \n', e_SUB);    

        %% Compute WLS (stable) [A, B] solution
        % fprintf('Computing stable A solution using WLS ... \n');
        % 
        % Pnew = [X_(:,1), Y_];
        % [U_wls,S_wls,V_wls] = svd(Pnew,0);
        % 
        % n = stateNum;
        % V_wls = V_wls(:,1:n);
        % S_wls = S_wls(1:n,1:n);
        % U_wls = U_wls(:,1:n);
        % 
        % timeWLS = tic;
        % [A_WLS, ~, ~, ~] = learnWLSmodel(V_wls,S_wls,1,0);
        % tEnd = toc(timeWLS);
        % 
        % e_WLS = norm(S_wls*V_wls(2:end,:)' - A_WLS * S_wls*V_wls(1:end-1,:)', 'fro')^2/2;
        % maxeval_WLS = max(abs(eig(A_WLS)));
        % if maxeval_WLS <= 1 
        %     WLS_stability(seq+1, stateNum - 2) = 1;
        % end
        % WLS_error(seq+1, stateNum - 2) = e_WLS;
        % WLS_time(seq+1, stateNum - 2) = tEnd;
        % 
        % save([save_directory, 'WLS_time.mat'], 'WLS_time');
        % save([save_directory, 'WLS_error.mat'], 'WLS_error');
        % save([save_directory, 'WLS_stability.mat'], 'WLS_stability');
        % 
        % fprintf('    Max eigenvalue is : %.4f \n', max(abs(eig(A_WLS)) ));
        % fprintf('    Reconstruction error : %.5f \n', e_WLS);    
  % % clearvars -except system
  % 
  %       %% Compute CG (stable) [A, B] solution
        fprintf('Computing stable A using CG ... \n');

        timeCG = tic;
        [A_CG, ~, ~, ~] = learnCGModel(X_, Y_, 1, 0);
        tEnd = toc(timeCG);

        e_CG = norm(Y_ - A_CG*X_, 'fro')^2/2;
        maxeval_CG = max(abs(eig(A_CG)));
        if maxeval_CG <= 1 
            CG_stability(seq+1, stateNum - 2) = 1;
        end
        CG_error(seq+1, stateNum - 2) = e_CG;
        CG_time(seq+1, stateNum - 2) = tEnd;

        save([save_directory, 'CG_time.mat'], 'CG_time');
        save([save_directory, 'CG_error.mat'], 'CG_error');
        save([save_directory, 'CG_stability.mat'], 'CG_stability');

        fprintf('    Max eigenvalue is : %.4f \n', maxeval_CG)
        fprintf('    Reconstruction error : %.5f \n', e_CG);    

    end
end
