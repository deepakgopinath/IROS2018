function [aisHeart, aisBreath] = activeInfoStorage_AutoEmbed(knn)
    tic;
	
	% Add utilities to the path
	addpath('..');

	% Assumes the jar is two levels up - change this if this is not the case
	% Octave is happy to have the path added multiple times; I'm unsure if this is true for matlab
	javaaddpath('../../../infodynamics.jar');

	data = load('../../data/SFI-heartRate_breathVol_bloodOx.txt');
    % Restrict to the samples that Schreiber mentions:
	data = data(2350:3550,:);
	
	% Separate the data from each column:
	heart = data(:,1);
	chestVol = data(:,2);
	bloodOx = data(:,3);
	timeSteps = length(heart);
    
    fprintf('AIS for heart rate and breath rate for Kraskov estimation with %d samples:\n', timeSteps);
	%instantiate
    aisCalc=javaObject('infodynamics.measures.continuous.kraskov.ActiveInfoStorageCalculatorKraskov');
    %setProperties
    aisCalc.setProperty('k', sprintf('%d',knn));
    aisCalc.setProperty('NORMALISE', 'true');
    aisCalc.setProperty(aisCalc.PROP_AUTO_EMBED_METHOD, aisCalc.AUTO_EMBED_METHOD_RAGWITZ);
    aisCalc.setProperty(aisCalc.PROP_K_SEARCH_MAX, '15');
    aisCalc.setProperty(aisCalc.PROP_TAU_SEARCH_MAX, '5');
    
    aisCalc.initialise();
    aisCalc.setObservations(octaveToJavaDoubleArray(heart(1:timeSteps))); %AIS just for the heart time series.
    aisHeart = aisCalc.computeAverageLocalOfObservations();
    
    %retrieve the embedding using getProperty. 
    
    kUsedH = char(aisCalc.getProperty(aisCalc.K_PROP_NAME));
    kTauUsedH = char(aisCalc.getProperty(aisCalc.TAU_PROP_NAME));
    
    %For breath, reuse same calculator.
    
    aisCalc.setProperty('k', sprintf('%d',knn));
	aisCalc.setProperty('NORMALISE', 'true');
    aisCalc.setProperty(aisCalc.PROP_AUTO_EMBED_METHOD, aisCalc.AUTO_EMBED_METHOD_RAGWITZ);
    aisCalc.setProperty(aisCalc.PROP_K_SEARCH_MAX, '15');
    aisCalc.setProperty(aisCalc.PROP_TAU_SEARCH_MAX, '5');
    
    aisCalc.initialise(); %clear the stored pdfs. 
    aisCalc.setObservations(octaveToJavaDoubleArray(chestVol(1:timeSteps))); %AIS just for the heart time series.
    aisBreath = aisCalc.computeAverageLocalOfObservations();
    
    kUsedB = char(aisCalc.getProperty(aisCalc.K_PROP_NAME));
    kTauUsedB = char(aisCalc.getProperty(aisCalc.K_TAU_PROP_NAME));
    
    totaltime = toc;
	fprintf('Total runtime was %.1f sec\n', totaltime);
    

end