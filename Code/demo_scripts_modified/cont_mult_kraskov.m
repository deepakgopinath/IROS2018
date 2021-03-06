
% = Example 9 - Transfer entropy on continuous multivariate data using Kraskov estimators =

% Transfer entropy (TE) calculation on multivariate continuous-valued data using the Kraskov-estimator TE calculator.
clear all; clc; close all;
%%
% Change location of jar to match yours:
javaaddpath('/home/deepak/Desktop/Research/PaperSubmissions/IROS2018/InfoDynamics/infodynamics.jar')

% Generate some random normalised data.
numObservations = 10000;
covariance=0.4; 

% Define the dimension of the states of the RVs
sourceDim = 2;  
destDim = 3;
zero_ind = datasample(1:10000, 9950, 'Replace', false);
sourceMVArray = randn(numObservations, sourceDim);
sourceMVArray(zero_ind, :) = 0;
% Set first two columns of dest to copy source values
destMVArray  = [zeros(1,sourceDim); covariance*(sourceMVArray(1:numObservations-1,:)) + (1-covariance)*randn(numObservations-1, sourceDim)];
% Set a third colum to be randomised
destMVArray(:,3) = randn(numObservations, 1);
zero_ind = datasample(1:10000, 9900, 'Replace', false);
destMVArray(zero_ind, :) = 0;

sourceMVArray2= randn(numObservations, sourceDim); % Uncorrelated source

% Create a TE calculator and run it:
teCalc=javaObject('infodynamics.measures.continuous.kraskov.TransferEntropyCalculatorMultiVariateKraskov');
teCalc.initialise(1,sourceDim,destDim); % Use history length 1 (Schreiber k=1)
teCalc.setProperty('k', '4'); % Use Kraskov parameter K=4 for 4 nearest points
teCalc.setObservations(octaveToJavaDoubleMatrix(sourceMVArray), octaveToJavaDoubleMatrix(destMVArray));
% Perform calculation with correlated source:
result = teCalc.computeAverageLocalOfObservations();
local_TE = teCalc.computeLocalOfPreviousObservations();
figure;
plot(sourceMVArray, 'r', 'LineWidth', 1.5); hold on; plot(destMVArray, 'b', 'LineWidth',1.5);
plot(local_TE);
% Note that the calculation is a random variable (because the generated
%  data is a set of random variables) - the result will be of the order
%  of what we expect, but not exactly equal to it; in fact, there will
%  be some variance around it. It will probably be biased down here
%  due to small correlations between the supposedly uncorrelated variables.
fprintf('TE result %.4f nats; expected to be close to %.4f nats for the two correlated Gaussians\n', ...
result, 2*log(1/(1-covariance^2)));

% Perform calculation with uncorrelated source:
teCalc.initialise(1,sourceDim,destDim); % Initialise leaving the parameters the same
teCalc.setObservations(octaveToJavaDoubleMatrix(sourceMVArray2), octaveToJavaDoubleMatrix(destMVArray));
result2 = teCalc.computeAverageLocalOfObservations();
fprintf('TE result %.4f nats; expected to be close to 0 nats for these uncorrelated Gaussians\n', result2);
clear teCalc

