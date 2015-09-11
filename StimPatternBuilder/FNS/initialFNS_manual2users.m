% Initialize Functional Neuromuscular Stimulation Patterns
% (for manual switch-trigger FNS gait)
% by Thomas Bulea based on original code by Curtis S. To
% date created: 022211
% date modified: 022311
%--------------------------------------------------------------------------
% The stimulation patterns are extracted from the .txt files named,
% ['SC' 'subject ID' '_' 'activity name' '_' 'stimulation parameter'
% '.txt'].  These stimulation patterns are profiled using
% VortexConvert_gui#.m.  The data from the .txt files are concatenated into
% matrices containing the stimulation percentages and pulse widths for
% standing, walking (by left/right steps), and sitting.  Each matrix is
% transposed and resized into a column vector
% ([muscle channel 1; muscle channel 2; ... muscle channel n]) for use in 
% the Simulink model.
%
% This m-file also reformats the input stimulation parameter of two 
% different users (Subject AA (user 1) and Subject BB (user 2)). AA has 16 
% stimulation channels and BB has 24 stimulation channels.  Thus, the input  
% AA stim is a 16 element vector and the input BB stim is a 24 element 
% vector.  The number of stimulation channels is 24 (nchan = max number of
% channels capable for the percutaneous external control unit) regardless
% of subject.  The output reformatted stim signal is a 24 element vector.
% The empty channels are zeroed out.  This m-file also reformats the muscle
% action matrix (MA) for accordingly.  The variable chstim = stimulation 
% channel number vector (i.e. for subject AA chstim = [1:8,13:20] and for 
% subject BB chstim = [1:24].
%
% This m-file sets the duration for each activity (i.e. sit, stand, and 
% step [left right]) with respect to the user.
%
% The FNS controller is capable of rudamentry interpulse interval
% modulation.  By default, the stimulation frequency (= ECU sampling  
% frequency)is halved by interleaving zeros as the value of ever other 
% pulse width.  When IPI modulation is turned on, this interleaving of 
% zeros is bypassed.  Thus, the frequency is increased to the ECU sampling 
% frequwncy of 33 Hz.  This  m-file calls FNSC_IPIpattern.m to set whether 
% the IPI of each channel will undergo modulation or not 
% (0 = no modulation; 1 = modulation) and the range (%) during each
% activity at with the IPI will be modulated.
%--------------------------------------------------------------------------

% Load stimulation pulse width values:
stand_values = load(['SC' subject '_stand_PW.txt']);
lstep_values = load(['SC' subject '_lstep_PW.txt']);
rstep_values = load(['SC' subject '_rstep_PW.txt']);
sit_values = load(['SC' subject '_sit_PW.txt']);
roff_values = load(['SC' subject '_roff_PW.txt']);
ldown_values = load(['SC' subject '_ldown_PW.txt']);

if subject == 'AA'  % user ID
    % Actvity durations (verified 2.21.11):
    dtsit = 1.5;              % sit duration (s)
    dtstand = 1.95;         % stand duration (s)
    dtstep = [1.325 1.555]; % step duration [left right] (s)
    dtstepdown = [2.70 2.60];   %stair descent step duration [right left] (s)
    % Reformat subject AA (user 1) muscle action matrix:
    MArf = zeros(nchan,3);  % initialize reformatted muscle action matrix
    for n = 1:length(chstim)
        MArf(chstim(n),:) = MA(n,:);
    end
    MA = MArf;
    
    % Set stimulation channel interpulse interval modulation states:
    % Verified on 2.22.11
    ipisit = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
    ipistand = [1 0 1 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0];
    ipilstep = [0 1 0 0 0 1 1 1 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0];
    ipirstep = [0 0 1 1 1 0 0 0 0 0 0 0 0 1 0 0 0 1 1 1 0 0 0 0];
    ipiroff = [0 0 1 1 1 0 0 0 0 0 0 0 0 1 0 0 0 1 1 1 0 0 0 0];
    ipildown = [0 1 0 0 0 1 1 1 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0];
    
    % Set IPI modulation range (%) for each activity:
    ipisitrange = [zeros(24,1) zeros(24,1)];
    ipistandrange = [0*ones(24,1) 95.0000*ones(24,1)];
    % Left Step Range
    ipilsteprange = [16.2264*ones(24,1) 66.7925*ones(24,1)];
    %Extend for Tensor 
    ipilsteprange(6,:) = [16.2264 75.849];
    %Right Step Range
    ipirsteprange = [18.3280*ones(24,1) 64.6302*ones(24,1)];
    %Extend for Tensor
    ipirsteprange(10,:) = [18.3280 79.100];
    %Stair Descent
    ipiroffrange = [zeros(24,1) zeros(24,1)];
    ipildownrange = [zeros(24,1) zeros(24,1)];
    
elseif subject == 'BB'  % user ID
    % Actvity durations (verified 2.21.11):
    dtsit = 2.05;           % sit duration (s)
    dtstand = 2.00;         % stand duration (s)
    dtstep = [0.98 1.00]; % step duration [left right] (s)
    
    % Reformat subject BB (user 2) muscle action matrix:
    MArf = zeros(nchan,3);  % initialize reformatted muscle action matrix
    for n = 1:length(chstim)
        MArf(chstim(n),:) = MA(n,:);
    end
    MA = MArf;
    
    % Set stimulation channel interpulse interval modulation states and
    % IPI modulation range (%) for each activity:
    % Verified on 2.22.11
    % During walking, allowed IPI to increase to TA/Illios instead of
    % Hams/gluts
%     FNSC_IPIpattern
    ipisit = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
    ipistand = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
    ipilstep = [0 0 0 0 0 0 1 1 1 1 1 0 0 0 0 1 0 0 0 0 0 0 1 0];
    ipirstep = [0 0 1 0 1 0 0 0 0 0 0 0 0 0 0 0 1 0 1 1 1 1 0 1];
    
    % Set IPI modulation range (%) for each activity:
    ipisitrange = [zeros(24,1) zeros(24,1)];
    ipistandrange = [zeros(24,1) zeros(24,1)];
    %Left Step (iliopsoas and TA)
    ipilsteprange = [40.8163*ones(24,1) 50.6122*ones(24,1)];
    %extend range for left tensor 
    ipilsteprange(7,:) = [40.8163 91.837];
    %change for right glut max and ham
    ipilsteprange(16,:) = [85.306 96.735];
    ipilsteprange(23,:) = [85.306 96.735];
    %Right Step (iliopsoas and TA)
    ipirsteprange = [37.6*ones(24,1) 56*ones(24,1)]; 
    %extend for right tensor
    ipirsteprange(19,:) = [37.6 91];
    %shorten for right Gas/Sol
    ipirsteprange(24,:) = [0  37.6];
    %shorten for left glut max and left ham
    ipirsteprange(3,:) = [70.3 83.8];
    ipirsteprange(5,:) = [70.3 83.8];
    
    
        
end

% Concatenate left step stimulation patterns into single matrix:
% Reformat Vortex pulse width (extracted tics):
stimx = lstep_values;
for n = 1:length(chstim)
    Px(n,:) = stimx(6*n-5:6*n,1);  	% Vortex extracted percentage tic values
    PWx(n,:) = stimx(6*n-5:6*n,2);	% Vortex extracted pulse width tic values (ms)
end

P = zeros(nchan, size(Px,2)); % initialize percentage
for n = 1:length(chstim)
    P(chstim(n),:) = Px(n,:);
end
GCl = P;    % left step percentage gait cycle

PW = zeros(nchan, size(PWx,2)); % initialize pulse width (ms)
for n = 1:length(chstim)
    PW(chstim(n),:) = PWx(n,:);
end
PWl = PW;    % left step pulse width (ms)

maxPW(:,1) = max(PWl,[],2);  % determine max pulse width levels for each channel

% Concatenate right step stimulation patterns into single matrix:
% Reformat Vortex pulse width (extracted tics):
stimx = rstep_values;
for n = 1:length(chstim)
    Px(n,:) = stimx(6*n-5:6*n,1);  	% Vortex extracted percentage tic values
    PWx(n,:) = stimx(6*n-5:6*n,2);	% Vortex extracted pulse width tic values (ms)
end

P = zeros(nchan, size(Px,2)); % initialize percentage
for n = 1:length(chstim)
    P(chstim(n),:) = Px(n,:);
end
GCr = P;    % right step percentage gait cycle

PW = zeros(nchan, size(PWx,2)); % initialize pulse width (ms)
for n = 1:length(chstim)
    PW(chstim(n),:) = PWx(n,:);
end
PWr = PW;    % right step pulse width (ms)

maxPW(:,2) = max(PWr,[],2);	% determine max pulse width levels for each channel

% Concatentate stand stimulation patterns into single matrix:
% Reformat Vortex pulse width (extracted tics):
stimx = stand_values;
for n = 1:length(chstim)
    Px(n,:) = stimx(6*n-5:6*n,1);  	% Vortex extracted percentage tic values
    PWx(n,:) = stimx(6*n-5:6*n,2);	% Vortex extracted pulse width tic values (ms)
end

P = zeros(nchan, size(Px,2)); % initialize percentage
for n = 1:length(chstim)
    P(chstim(n),:) = Px(n,:);
end
standstimp = P;    % stand percentage

PW = zeros(nchan, size(PWx,2)); % initialize pulse width (ms)
for n = 1:length(chstim)
    PW(chstim(n),:) = PWx(n,:);
end
standstimpw = PW;    % stand pulse width (ms)

maxPW(:,3) = max(standstimpw,[],2);	% determine max pulse width levels for each channel

% Concatentate sit stimulation patterns into single matrix:
% Reformat Vortex pulse width (extracted tics):
stimx = sit_values;
for n = 1:length(chstim)
    Px(n,:) = stimx(6*n-5:6*n,1);  	% Vortex extracted percentage tic values
    PWx(n,:) = stimx(6*n-5:6*n,2);	% Vortex extracted pulse width tic values (ms)
end

P = zeros(nchan, size(Px,2)); % initialize percentage
for n = 1:length(chstim)
    P(chstim(n),:) = Px(n,:);
end
sitstimp = P;    % sit percentage

PW = zeros(nchan, size(PWx,2)); % initialize pulse width (ms)
for n = 1:length(chstim)
    PW(chstim(n),:) = PWx(n,:);
end
sitstimpw = PW;    % sit pulse width (ms)

maxPW(:,4) = max(sitstimpw,[],2);	% determine max pulse width levels for each channel

% Concatenate right off (descent) stimulation patterns into single matrix:
% Reformat Vortex pulse width (extracted tics):
stimx = roff_values;
for n = 1:length(chstim)
    Px(n,:) = stimx(6*n-5:6*n,1);  	% Vortex extracted percentage tic values
    PWx(n,:) = stimx(6*n-5:6*n,2);	% Vortex extracted pulse width tic values (ms)
end

P = zeros(nchan, size(Px,2)); % initialize percentage
for n = 1:length(chstim)
    P(chstim(n),:) = Px(n,:);
end
GCro = P;    % left step percentage gait cycle

PW = zeros(nchan, size(PWx,2)); % initialize pulse width (ms)
for n = 1:length(chstim)
    PW(chstim(n),:) = PWx(n,:);
end
PWro = PW;    % left step pulse width (ms)

maxPW(:,5) = max(PWro,[],2);  % determine max pulse width levels for each channel

% Concatenate left down (descent) stimulation patterns into single matrix:
% Reformat Vortex pulse width (extracted tics):
stimx = ldown_values;
for n = 1:length(chstim)
    Px(n,:) = stimx(6*n-5:6*n,1);  	% Vortex extracted percentage tic values
    PWx(n,:) = stimx(6*n-5:6*n,2);	% Vortex extracted pulse width tic values (ms)
end

P = zeros(nchan, size(Px,2)); % initialize percentage
for n = 1:length(chstim)
    P(chstim(n),:) = Px(n,:);
end
GCld = P;    % left step percentage gait cycle

PW = zeros(nchan, size(PWx,2)); % initialize pulse width (ms)
for n = 1:length(chstim)
    PW(chstim(n),:) = PWx(n,:);
end
PWld = PW;    % left step pulse width (ms)

maxPW(:,6) = max(PWld,[],2);  % determine max pulse width levels for each channel

% Determine absolute max pulse width levels for each channel:
maxPW = max(maxPW,[],2);

% Resize matrices for use in Simulink model:
% left step percentage gait cycle
GCl = GCl';                     % transpose (make into 6 rows by 24 columns)
GCl = GCl(:);                   % resize to vector  (stack columns underneath one another)

% left step pulse width (ms)
PWl = PWl';                     % transpose
PWl = PWl(:);                   % resize to vector

% right step percentage gait cycle
GCr = GCr';                     % transpose
GCr = GCr(:);                   % resize to vector 

% right step pulse width (ms)
PWr = PWr';                     % transpose
PWr = PWr(:);                   % resize to vector

% left step percentage gait cycle
GCro = GCro';                     % transpose
GCro = GCro(:);                   % resize to vector

% left step pulse width (ms)
PWro = PWro';                     % transpose
PWro = PWro(:);                   % resize to vector

% left step percentage gait cycle
GCld = GCld';                     % transpose (make into 6 rows by 24 columns)
GCld = GCld(:);                   % resize to vector  (stack columns underneath one another)

% left step pulse width (ms)
PWld = PWld';                     % transpose
PWld = PWld(:);                   % resize to vector
% GC = [GCl,GCr];                 % combine left and right steps
% GC = GC(:);                     % resize to vector
% PW = [PWl,PWr];                 % combine left and right steps
% PW = PW(:);                     % resize to vector

% stand percentage
standstimp = standstimp';       % transpose
standstimp = standstimp(:);     % resize to vector 

% stand pulse width (ms)
standstimpw = standstimpw';     % transpose
standstimpw = standstimpw(:);   % resize to vector

% sit percentage
sitstimp = sitstimp';           % transpose
sitstimp = sitstimp(:);         % resize to vector 

% sit pulse width (ms)
sitstimpw = sitstimpw';         % transpose
sitstimpw = sitstimpw(:);       % resize to vector