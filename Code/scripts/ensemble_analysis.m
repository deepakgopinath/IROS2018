clear all; clc; close all;
%%
javaaddpath('../../InfoDynamics/infodynamics.jar');
load('data_file.mat');
num_trials = size(alpha_all, 1);
num_sub = 8; %foir the time being just look at one subject
trials_per_sub = 32*ones(num_sub,1); trials_per_sub(5) = 31;

%instantiate the calculators
aisCalcS=javaObject('infodynamics.measures.continuous.kraskov.ActiveInfoStorageCalculatorMultiVariateKraskov');
aisCalcD=javaObject('infodynamics.measures.continuous.kraskov.ActiveInfoStorageCalculatorMultiVariateKraskov');
teCalc=javaObject('infodynamics.measures.continuous.kraskov.TransferEntropyCalculatorMultiVariateKraskov');
    
j_id = 1; ha_id = 2; re_id = 1; po_id = 2;
reaching_goal_pos = [   0.328, -0.516, 0.1;
                        -0.116, -0.579, 0.132;
                        0.124, -0.531, 0.442;
                        -0.346, -0.389, 0.220;
                        -0.376, -0.180, 0.102
                        ]';
pouring_goal_pos = [    0.3314, -0.206, 0.1;
                        0.306, -0.550, 0.256;
                        -0.05, -0.627, 0.225;
                        -0.370, -0.55504, 0.3955
                        ]';
                    
sub_markers = cumsum(trials_per_sub); init_sub_markers = sub_markers - 31; init_sub_markers(5) = 129;
knn = 4;
%%
for jj = 1:num_sub %for every subject
    %set properties for each AIS calculator before adding observations
%     aisCalcD.setProperty(aisCalcD.PROP_AUTO_EMBED_METHOD, aisCalcD.AUTO_EMBED_METHOD_RAGWITZ);
%     aisCalcD.setProperty(aisCalcD.PROP_K_SEARCH_MAX, '10');
%     aisCalcD.setProperty(aisCalcD.PROP_TAU_SEARCH_MAX, '4');
%     aisCalcD.setProperty('k', sprintf('%d',knn))
%     aisCalcD.setProperty('NORMALISE', 'true');
%     aisCalcD.initialise(6);
%     aisCalcD.startAddObservations();
%     
%     aisCalcS.setProperty(aisCalcS.PROP_AUTO_EMBED_METHOD, aisCalcS.AUTO_EMBED_METHOD_RAGWITZ);
%     aisCalcS.setProperty(aisCalcS.PROP_K_SEARCH_MAX, '10');
%     aisCalcS.setProperty(aisCalcS.PROP_TAU_SEARCH_MAX, '4');
%     aisCalcS.setProperty('k', sprintf('%d',knn))
%     aisCalcS.setProperty('NORMALISE', 'true');
%     aisCalcS.initialise(6);
%     aisCalcS.startAddObservations();
%     
%     
%     for i=init_sub_markers(jj):sub_markers(jj) %all trials for a specific subjetc. 
%         if interface_all{i} == ha_id || task_all{i} == po_id %if it is headarray or pouring skip. only interested in joystick and reaching
%             continue
%         end
%         %lets look at goal velocities towards intended goal multiplied by alpha
%         %and user velocities. 
% 
%         alpha = alpha_all{i}; %alpha = (alpha > 0)*1;
%         gv = gv_all{i}; gv =  gv'; gv(:, end-1:end) = []; %remove time index row. 
%         gv = repmat(alpha, 1, size(gv, 2)).*gv; % the portion of robot autonomy that the human sees. 
% 
%         gp = gp_all{i}; gp = gp(ig_all{i}, :)';
%         uv = uv_all{i}; uv = uv'; uv = uv(:, 1:end-1); %user velocity from joystick
%         ts = uv_all{i}(end, :)'; %time stamps
%         ms = ms_all{i}; %mode switch timings. 
% 
%         %robot to human
%         sourceMVArray = gv;
%         destMVArray = uv;
%         sourceDim = size(sourceMVArray, 2);  
%         destDim = size(destMVArray, 2);
%         knn = 4; %neighbors for Kraskov estimation. fixed according to KSG original paper. 
% 
%         %human to robot.
%     %     sourceMVArray = uv;
%     %     destMVArray = gp;
%     %     sourceDim = size(sourceMVArray, 2);  
%     %     destDim = size(destMVArray, 2);
%     %     knn = 4;
% 
%         %% COMPUTE DESTINATION EMBEDDING
%         aisCalcD.addObservations(octaveToJavaDoubleMatrix(destMVArray));
%         
%         %% COMPUTE SOURCE EMBEDDING
%         aisCalcS.addObservations(octaveToJavaDoubleMatrix(sourceMVArray));
%     end
    %%
%     aisCalcD.finaliseAddObservations();
%     ais = aisCalcD.computeAverageLocalOfObservations();
%     fprintf('The AIS in destination signal is %f\n', ais);
%     kUsedD = char(aisCalcD.getProperty(aisCalcD.K_PROP_NAME)); disp(kUsedD);
%     kTauUsedD = char(aisCalcD.getProperty(aisCalcD.TAU_PROP_NAME));
%     
%     
%     aisCalcS.finaliseAddObservations();
%     ais = aisCalcS.computeAverageLocalOfObservations();
%     fprintf('The AIS in source signal is %f\n', ais);
%     kUsedS = char(aisCalcD.getProperty(aisCalcD.K_PROP_NAME)); disp(kUsedS);
%     kTauUsedS = char(aisCalcD.getProperty(aisCalcD.TAU_PROP_NAME));
    
    %% Compute TE. USe ensemble approach
    delay = 1;
%     teCalc.initialise(6, 6, str2num(kUsedS), str2num(kTauUsedS), str2num(kUsedD), str2num(kTauUsedD), delay);
    teCalc.initialise(6, 6, 1,1,1,1, delay);
    teCalc.setProperty('ALG_NUM', '1');
    teCalc.setProperty('NORMALISE', 'true');   
    teCalc.setProperty('NOISE_LEVEL_TO_ADD', '0.0000001');
    teCalc.setProperty('k', '4'); % Use Kraskov parameter K=4 for 4 nearest points
    teCalc.startAddObservations();
    
    for i=init_sub_markers(jj):sub_markers(jj)
        if interface_all{i} == ha_id || task_all{i} == po_id %if it is headarray or pouring skip. only interested in joystick and reaching
            continue
        end
        %lets look at goal velocities towards intended goal multiplied by alpha
        %and user velocities. 

        alpha = alpha_all{i}; %alpha = (alpha > 0)*1;
        gv = gv_all{i}; gv =  gv'; gv(:, end-1:end) = []; %remove time index row. 
        gv = repmat(alpha, 1, size(gv, 2)).*gv; % the portion of robot autonomy that the human sees. 

        gp = gp_all{i}; gp = gp(ig_all{i}, :)';
        uv = uv_all{i}; uv = uv'; uv = uv(:, 1:end-1); %user velocity from joystick
        ts = uv_all{i}(end, :)'; %time stamps
        ms = ms_all{i}; %mode switch timings. 

        %robot to human
        sourceMVArray = gv;
        destMVArray = uv;
        sourceDim = size(sourceMVArray, 2);  
        destDim = size(destMVArray, 2);
        knn = 4; %neighbors for Kraskov estimation. fixed according to KSG original paper. 

        %human to robot.
    %     sourceMVArray = uv;
    %     destMVArray = gp;
    %     sourceDim = size(sourceMVArray, 2);  
    %     destDim = size(destMVArray, 2);
    %     knn = 4;
        teCalc.addObservations(octaveToJavaDoubleMatrix(sourceMVArray), octaveToJavaDoubleMatrix(destMVArray));
    end
    teCalc.finaliseAddObservations();
    result = teCalc.computeAverageLocalOfObservations();
    fprintf('The TE is for subject %d is %f\n', jj, result);
%     local_TE = teCalc.computeLocalOfPreviousObservations();
    repeats = 1000;
    nullDist = teCalc.computeSignificance(repeats);
    empCalc=javaObject('infodynamics.utils.EmpiricalMeasurementDistribution', nullDist.distribution, result);
    fprintf('The p-value is %f for subject %d\n', empCalc.pValue, jj);
    distribution = javaMatrixToOctave(nullDist.distribution);
    pvalue = sum(distribution >= nullDist.actualValue)/length(distribution);
    
% 
end
%%

            %%
%         figure; hold on; grid on;
%         plot(ts, local_TE, 'g', 'LineWidth', 2.0); 
%         plot(ts, sourceMVArray, 'r', 'LineWidth', 1.5);  plot(ts, destMVArray, 'b', 'LineWidth', 1.5);
%         plot(ts, alpha_all{i}, 'm', 'LineWidth', 1.5);
%         scatter(ms, 0.5*ones(length(ms), 1), 'filled');
%         close all;

        %% COMPUTE TE

%         delay = 1;
%         teCalc.initialise(sourceDim, destDim, str2num(kUsedS), str2num(kTauUsedS), str2num(kUsedD), str2num(kTauUsedD), delay);
%     %     teCalc.initialise(sourceDim, destDim, 1,1,1,1, delay);
%         teCalc.setProperty('ALG_NUM', '1');
%     %     teCalc.setProperty('NORMALISE', 'true');   
%         teCalc.setProperty('NOISE_LEVEL_TO_ADD', '0.0000001');
%         teCalc.setProperty('k', '4'); % Use Kraskov parameter K=4 for 4 nearest points
%         teCalc.setObservations(octaveToJavaDoubleMatrix(sourceMVArray), octaveToJavaDoubleMatrix(destMVArray));
%         result = teCalc.computeAverageLocalOfObservations();
%         fprintf('The TE is %f\n', result);
%         local_TE = teCalc.computeLocalOfPreviousObservations();
% 
%         repeats = 100;
%         nullDist = teCalc.computeSignificance(repeats);
%         empCalc=javaObject('infodynamics.utils.EmpiricalMeasurementDistribution', nullDist.distribution, result);
%         fprintf('The p-value is %f\n', empCalc.pValue);
%         distribution = javaMatrixToOctave(nullDist.distribution);
%         pvalue = sum(distribution >= nullDist.actualValue)/length(distribution);

% aisCalcS.finaliseAddObservations();
% ais = aisCalcS.computeAverageLocalOfObservations();
% fprintf('The AIS in source signal is %f\n', ais);
% kUsedS = char(aisCalcD.getProperty(aisCalcD.K_PROP_NAME)); disp(kUsedS);
% kTauUsedS = char(aisCalcD.getProperty(aisCalcD.TAU_PROP_NAME));
    
