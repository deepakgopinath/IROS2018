%%
%%  Java Information Dynamics Toolkit (JIDT)
%%  Copyright (C) 2014, Viola Priesemann, Joseph T. Lizier
%%  
%%  This program is free software: you can redistribute it and/or modify
%%  it under the terms of the GNU General Public License as published by
%%  the Free Software Foundation, either version 3 of the License, or
%%  (at your option) any later version.
%%  
%%  This program is distributed in the hope that it will be useful,
%%  but WITHOUT ANY WARRANTY; without even the implied warranty of
%%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%%  GNU General Public License for more details.
%%  
%%  You should have received a copy of the GNU General Public License
%%  along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%

% = Example 9 - Transfer entropy on continuous multivariate data using Kraskov estimators =

% Transfer entropy (TE) calculation on multivariate continuous-valued data using the Kraskov-estimator TE calculator.

% Change location of jar to match yours:
javaaddpath('../../infodynamics.jar')

% Generate some random normalised data.
numObservations = 10000;
covariance=0.4; 

% Define the dimension of the states of the RVs
sourceDim = 2;  
destDim = 3;

sourceMVArray = randn(numObservations, sourceDim);
% Set first two columns of dest to copy source values
destMVArray  = [zeros(1,sourceDim); covariance*(sourceMVArray(1:numObservations-1,:)) + (1-covariance)*randn(numObservations-1, sourceDim)];
% Set a third colum to be randomised
destMVArray(:,3) = randn(numObservations, 1);
sourceMVArray2= randn(numObservations, sourceDim); % Uncorrelated source

knn = 4;
%Create multivariate AIS calculator
aisCalc = javaObject('infodynamics.measures.continuous.kraskov.ActiveInfoStorageCalculatorMultiVariateKraskov');

%% Compute AIS for destination to determined embedding, 9 for k and 2 for tau
aisCalc.setProperty(aisCalc.PROP_AUTO_EMBED_METHOD, aisCalc.AUTO_EMBED_METHOD_RAGWITZ);
aisCalc.setProperty(aisCalc.PROP_K_SEARCH_MAX, '10');
aisCalc.setProperty(aisCalc.PROP_TAU_SEARCH_MAX, '4');
aisCalc.setProperty('k', sprintf('%d',knn))
% aisCalc.setProperty('NORMALISE', 'true');
aisCalc.initialise(destDim);
aisCalc.setObservations(octaveToJavaDoubleMatrix(destMVArray));
ais = aisCalc.computeAverageLocalOfObservations();

kUsedD = char(aisCalc.getProperty(aisCalc.K_PROP_NAME));
kTauUsedD = char(aisCalc.getProperty(aisCalc.TAU_PROP_NAME));


%% AIS FOR SOURCE
aisCalc.setProperty(aisCalc.PROP_AUTO_EMBED_METHOD, aisCalc.AUTO_EMBED_METHOD_RAGWITZ);
aisCalc.setProperty(aisCalc.PROP_K_SEARCH_MAX, '10');
aisCalc.setProperty(aisCalc.PROP_TAU_SEARCH_MAX, '4');
aisCalc.setProperty('k', sprintf('%d',knn))
% aisCalc.setProperty('NORMALISE', 'true');
aisCalc.initialise(sourceDim);
aisCalc.setObservations(octaveToJavaDoubleMatrix(sourceMVArray));
ais = aisCalc.computeAverageLocalOfObservations;

kUsedS = char(aisCalc.getProperty(aisCalc.K_PROP_NAME));
kTauUsedS = char(aisCalc.getProperty(aisCalc.TAU_PROP_NAME));

%% TE between source and destination
delay = 1;
teCalc=javaObject('infodynamics.measures.continuous.kraskov.TransferEntropyCalculatorMultiVariateKraskov');
teCalc.initialise(sourceDim, destDim, str2num(kUsedS), str2num(kTauUsedS), str2num(kUsedD), str2num(kTauUsedD), delay);
teCalc.setProperty('k', '4'); % Use Kraskov parameter K=4 for 4 nearest points
teCalc.setObservations(octaveToJavaDoubleMatrix(sourceMVArray), octaveToJavaDoubleMatrix(destMVArray));
result = teCalc.computeAverageLocalOfObservations();

%%
fprintf('TE result %.4f nats; expected to be close to %.4f nats for the two correlated Gaussians\n', ...
result, 2*log(1/(1-covariance^2)));

