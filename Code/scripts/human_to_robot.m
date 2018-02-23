clear all; clc; close all;
%%
javaaddpath('../../InfoDynamics/infodynamics.jar');
load('new_data_file.mat');
num_trials = size(alpha_all, 1);
num_sub = 8; %foir the time being just look at one subject
trials_per_sub = 32*ones(num_sub,1); trials_per_sub(5) = 31;

%instantiate the calculators
aisCalcS=javaObject('infodynamics.measures.continuous.kraskov.ActiveInfoStorageCalculatorMultiVariateKraskov');
aisCalcD=javaObject('infodynamics.measures.continuous.kraskov.ActiveInfoStorageCalculatorMultiVariateKraskov');
teCalc=javaObject('infodynamics.measures.continuous.kraskov.TransferEntropyCalculatorMultiVariateKraskov');
    
goal_pos = cell(2, 1);
goal_pos{1} = [   0.328, -0.516, 0.1;
                        -0.116, -0.579, 0.132;
                        0.124, -0.531, 0.442;
                        -0.346, -0.389, 0.220;
                        -0.376, -0.180, 0.102
                        ]';
goal_pos{2} = [0.3314, -0.206, 0.1;
                        0.306, -0.550, 0.256;
                        -0.05, -0.627, 0.225;
                        -0.370, -0.55504, 0.3955
                        ]';
%%
num_trials = size(alpha_all, 1);
num_sub = 8; %foir the time being just look at one subject
trials_per_sub = 32*ones(num_sub,1); trials_per_sub(5) = 31;
sub_markers = cumsum(trials_per_sub); init_sub_markers = sub_markers - 31; init_sub_markers(5) = 129;
knn = 4;
j_id = 1; ha_id = 2; re_id = 1; po_id = 2;
global_TE_list = zeros(length(alpha_all), 1);
%%
for jj=1:num_sub
    for i=init_sub_markers(jj):sub_markers(jj) %all trials for a specific subjetc. 
        if interface_all{i} == ha_id %if it is headarray or pouring skip. only interested in joystick and reaching
            continue
        end
         if ms_all{i} == -999 %if failed trial skip as well. 
            global_TE_list(i) = -999;
            continue;
         end
        
        alpha = alpha_all{i}; %alpha = (alpha > 0)*1;
        gv = gv_all{i};gv(end-1:end, :) = []; %remove time index row. 
%         gv = repmat(alpha, 1, size(gv, 2)).*gv; % the portion of robot autonomy that the human sees. 

        gp = gp_all{i}; gp = gp(ig_all{i}, :)'; %goal probability associated with intended goal. 
        uv = uv_all{i}; uv = uv(1:end-1, :);%user velocity from interface
        ts = uv_all{i}(end, :)'; %time stamps
        ms = ms_all{i}; %mode switch timings. 
        total_time = tt_all{i};
        task_id = task_all{i};
        ig_pos = goal_pos{task_id}(:, ig_all{i});
        robot_pos = pos_all{i};
        
        %compute directedness. 
        alignvec = repmat(ig_pos, 1, size(robot_pos, 2)) - robot_pos; normalignvec = realmin + sqrt(sum(alignvec.*alignvec));
        alignvec = alignvec./repmat(normalignvec, size(alignvec, 1), 1);
        transuv = uv(1:3, :); normtransuv = realmin + sqrt(sum(transuv.*transuv));
        transuv = transuv./repmat(normtransuv, size(transuv, 1), 1);
        directedness = sum(transuv.*alignvec)'; %normalized. 
        
        %compute agreement - trans
        transgv = gv(1:3, :); normtransgv = realmin + sqrt(sum(transgv.*transgv));
        transgv = transgv./repmat(normtransgv, size(transgv, 1), 1);
        
        transagreement = sum(transuv.*transgv);
        
        %compute agreement - rot
        rotuv = uv(4:6, :); normrotuv = realmin + sqrt(sum(rotuv.*rotuv));
        rotuv = rotuv./repmat(normrotuv, size(rotuv, 1), 1);
        rotgv = gv(4:6, :); normrotgv = realmin + sqrt(sum(rotgv.*rotgv));
        rotgv = rotgv./repmat(normrotgv, size(rotgv, 1), 1);
        rotagreement  = sum(rotuv.*rotgv);
        
        agreement = 0.5*(transagreement + rotagreement);
        
        
        %%
        sourceMVArray = uv'; %alignment
        destMVArray = alpha;
        sourceDim = size(sourceMVArray, 2);  
        destDim = size(destMVArray, 2);
        knn = 4;
        
        %% TE stuff
        delay = 1;
%         teCalc.initialise(sourceDim, destDim, str2num(kUsedS), str2num(kTauUsedS), str2num(kUsedD), str2num(kTauUsedD), delay);
        teCalc.initialise(sourceDim, destDim, 1,1,1,1, delay); %k=1, l=1, 
        teCalc.setProperty('ALG_NUM', '1');
        teCalc.setProperty('NORMALISE', 'true');   
        teCalc.setProperty('NOISE_LEVEL_TO_ADD', '0.0000001');
        teCalc.setProperty('k', '4'); % Use Kraskov parameter K=4 for 4 nearest points
        teCalc.setObservations(octaveToJavaDoubleMatrix(sourceMVArray), octaveToJavaDoubleMatrix(destMVArray));
        global_TE = teCalc.computeAverageLocalOfObservations();
        fprintf('The TE is %f\n', global_TE);
        global_TE_list(i) = global_TE;
        local_TE = teCalc.computeLocalOfPreviousObservations();

        %% figure plotting. 
        
%         figure; hold on; grid on;
% %         plot(ts, local_TE, 'g', 'LineWidth', 2.0); 
%         plot(ts, sourceMVArray, 'r', 'LineWidth', 1.5);  plot(ts, destMVArray, 'b', 'LineWidth', 1.5);
% %         plot(ts, alpha_all{i}, 'm', 'LineWidth', 1.5);
%         scatter(ms, 0.5*ones(length(ms), 1), 'filled');
%         
%         close all;
        
        
    end
end

%%
% clear all; clc; close all;
% %%
% load('TE_transagreement.mat'); load('TE_rotagreement.mat'); load('new_data_file.mat');
% %%
% pred = TE_transagreement(TE_transagreement~=0 & TE_transagreement~=-999);
% resp = cell2mat(tt_all(TE_transagreement~=0 & TE_transagreement~=-999));
% scatter(pred, resp, 'filled', 'r');
% 
% %%
% hold on;
% pred = TE_rotagreement(TE_rotagreement~=0 & TE_rotagreement~=-999);
% resp = cell2mat(tt_all(TE_rotagreement~=0 & TE_rotagreement~=-999));
% scatter(pred, resp, 'filled', 'b');
% [fitobject, gof] = fit(pred, resp, 'poly1', 'Robust', 'LAR');
% scatter(x, y, 'filled', 'r'); grid on; 