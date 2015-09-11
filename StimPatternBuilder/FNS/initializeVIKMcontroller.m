% initializeVIKMcontroller
% Initializes the VIKM and Data Acquisition Model titled:
% "xPC_VIKM_Controller"
% by Thomas C. Bulea
% date created: 121410
% data modified: 022111
% Call in the PreLoadFcn of Model Properties Callbacks of Simulink model.
%--------------------------------------------------------------------------

Ts = 0.050; % ECU sample time  %changed from Ts = 0.030
disp(['ECU sample time (Ts) = ' num2str(Ts) ' s']);
Tsx = 0.005; %added by SRC on 03.05.2013
tsx = 0.005; % exoskeleton/controller sample time
disp(['controller sample time (tsx) = ' num2str(tsx) ' s']);

surf_stim = 0;  %added by SRC on 03.05.2013, since not using surface stim

%added 03.05.2013
pitch = 50;   %settings for beeps/sound
volume = 250;
duration = 200;


subtype = 'ZZ'; % initialize subject type
while (subtype ~= 'AB') & (subtype ~= 'SC')
    subtype = input('Enter subject type (able-bodied (AB) or spinal cord injured (SC)): ','s');
end

subid = 'ZZ'; % initialize subject ID
if subtype == 'SC'
    while (subid ~= 'AA') & (subid ~= 'BB')
        subid = input('Enter subject ID (AA or BB): ','s');
        if subid == 'AA'
            subject = 'AA';
            % extract_values_subjAA;
            chstim = [1:8,13:20]; % stimulation channel number vector
            MA = load(['SCI' subject 'FNS_MA.txt']); % muscle action
        elseif subid == 'BB'
            subject = 'BB';
            % extract_values_subjBB;
            chstim = [1:24]; % stimulation channel number vector
            MA = load(['SCI' subject 'FNS_MA.txt']); % muscle action
        end
    end
else
    subtype = 'AB';
    subid = input('Enter subject ID #: ','s');
    subject = 'AA';
    % extract_values_subjAA;
    chstim = [1:8,13:20]; % stimulation channel number vector
    MA = load(['SCI' subject 'FNS_MA.txt']); % muscle action
end

nchan = 24; % number of stim channels

%Initialize Stimulation 
initialFNS_manual2users % initialize percutaneous stimulation patterns
if surf_stim == 1;	% used surface electrode channels
    initialFNS_2users_surf	% initialize surface stimulation patterns
end

kneestimtics = FNSkneetics(nchan,GCl,GCr,PWl,PWr,MA);
descentstimtics = [0.44 0.22];  

dtpoll = 0.06; % min duration of button poll
dtnopoll = 0.09; % min duration of no button poll

