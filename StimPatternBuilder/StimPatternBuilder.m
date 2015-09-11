% StimPatternBuilder.m - Load Stim Pattern to HNPv2 ECB.
% Created by Lu Li (lxl361@case), SEP, 2015.
% Version 1.0
% Online Doc: https://goo.gl/s20iH4
% Repo: https://github.com/lilulab/CwruStim_beta

%% Initialize
clear all;
path = pwd;
addpath([pwd '\FNS']);

%% Run FNS scripts
initializeVIKMcontroller
initialFNS_manual2users

%% Process data
% Determine absolute max pulse width levels for each channel:
stim_patterns.max_pw = uint8(maxPW);

% Resize matrices for use in Simulink model:
% left step percentage gait cycle
stim_patterns.step_lft_pct = uint8(GCl);

% left step pulse width (ms)
stim_patterns.step_lft_pw = uint8(PWl);

% right step percentage gait cycle
stim_patterns.step_rst_pct = uint8(GCr);

% right step pulse width (ms)
stim_patterns.step_rst_pw = uint8(PWr);

% right off percentage gait cycle
%GCro

% right off pulse width (ms)
%PWro

% left down percentage gait cycle
%GCld

% left down pulse width (ms)
%PWld

% stand percentage
stim_patterns.stand_pct = uint8(standstimp);

% stand pulse width (ms)
stim_patterns.stand_pw = uint8(standstimpw);

% sit percentage
stim_patterns.sit_pct = uint8(sitstimp);

% sit pulse width (ms)
stim_patterns.sit_pw = uint8(sitstimpw);


%% Generate head files
% open head file
fid = fopen('StimPattern.h', 'w');

% print a comment title, followed by a blank line
fprintf(fid, '// StimPattern.h - Head file for storage Stim Patterns.\n');
fprintf(fid, '// Created by Lu Li (lxl361@case), SEP, 2015.\n');
fprintf(fid, '// Version 1.0\n');
fprintf(fid, '// Repo: https://github.com/lilulab/CwruStim_beta\n\n');

% print include
fprintf(fid, '#include <stdint.h>;\n\n');

% print values
fprintf(fid, 'int8_t max_pw = [');
temp_size = size(stim_patterns.max_pw);
for i=1:temp_size(1)-1
    fprintf(fid, '%d, ', stim_patterns.max_pw(i));
end
fprintf(fid, '%d];\n\n', stim_patterns.max_pw(temp_size(1)));

fprintf(fid, '// File End\n\n');

fclose(fid);

print_c_array(123);

