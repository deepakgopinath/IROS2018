%%
%%  Java Information Dynamics Toolkit (JIDT)
%%  Copyright (C) 2012, Joseph T. Lizier
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

% = Example 1 - Transfer entropy on binary data =

% Simple transfer entropy (TE) calculation on binary data using the discrete TE calculator:

% Change location of jar to match yours:
% javaaddpath('../../infodynamics.jar');
javaaddpath('/home/deepak/Desktop/PaperSubmissions/IROS2018/InfoDynamics/infodynamics.jar')
% Generate some random binary data.
% Note that we need the *1 to make this a number not a Boolean,
%  otherwise this will not work (as it cannot match the method signature)
sourceArray=(rand(1000,1)>0.2)*1;  %number of rows is the number of time steps and number of columns is the dimension. This is univariate. 
destArray = [0; sourceArray(1:999)];
sourceArray2=(rand(1000,1)>0.2)*1;
% Create a TE calculator and run it:
teCalc=javaObject('infodynamics.measures.discrete.TransferEntropyCalculatorDiscrete', 2, 1); %first argument is the base - number of discrete levels in the variable and second argument is taregt embedding length, which is 'k' in Schreiber notation. 
%Note that this variable is NOT called 'k' in the class. Not to be confused
%with the member variable 'k' used in Kraskov classes which correspond to
%the number of nearest neighbors. 

teCalc.initialise();
% Since we have simple arrays of ints, we can directly pass these in:
teCalc.addObservations(sourceArray, destArray);
fprintf('For copied source, result should be close to 1 bit : ');
result = teCalc.computeAverageLocalOfObservations()

teCalc.initialise();
teCalc.addObservations(sourceArray2, destArray);
fprintf('For random source, result should be close to 0 bits: ');
result2 = teCalc.computeAverageLocalOfObservations()


