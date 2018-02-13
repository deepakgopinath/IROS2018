clear all; clc; close all;
%% LOAD DATA

subList = {'H3'};
total_subjects = length(subList);
trialList = 2;
phases = {'PH1','PH2'};
interfaces = {'J2', 'HA'};
tasks = {'RE','PO'};
assis = {'wo', 'on'};
task_order = {'RE'}; %Phase 1 tasks. 
trials_per_phase = 16;

%%
for i=1:total_subjects
    user = subList{i};
    trialId = trialList(i);
    fnames = dir(strcat('Data/',user));
    numfids = length(fnames);
    for j=3:numfids
        n = fnames(j).name;
        load(n);
        temp = n; n(end-3:end) = []; %remove .mat extension
        if length(n) == 12
            trialnum = str2double(n(end-1:end));
        else
            trialnum = str2double(n(end));
        end
        if strcmp(n(3:5), 'PH1')
            ph = 1;
        elseif strcmp(n(3:5), 'PH2')
            ph = 2;
        end
        subid = str2double(n(2)) - 1;
    end
    
end