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
fprintf(fid, '// StimPattern.h - Head file for storage Stim Patterns.\r\n');
fprintf(fid, '// Created by Lu Li (lxl361@case), SEP, 2015.\r\n');
fprintf(fid, '// Version 1.0\r\n');
fprintf(fid, '// Repo: https://github.com/lilulab/CwruStim_beta\r\n\r\n');

% print include
fprintf(fid, '#include <stdint.h>;\r\n\r\n');

% clear memory size counter.
Mem_size = 0;

% Print data array
% Mem_size = Mem_size + printCppArray( file_id, data_src, data_type )

% Determine absolute max pulse width levels for each channel:
Mem_size = Mem_size + printCppArray( fid, 'max_pw' , stim_patterns.max_pw, 'uint8_t' );

% left step percentage gait cycle
Mem_size = Mem_size + printCppArray( fid, 'step_lft_pct' , stim_patterns.step_lft_pct, 'uint8_t' );

% left step pulse width (ms)
Mem_size = Mem_size + printCppArray( fid, 'step_lft_pw' , stim_patterns.step_lft_pw, 'uint8_t' );

% right step percentage gait cycle
Mem_size = Mem_size + printCppArray( fid, 'step_rst_pct' , stim_patterns.step_rst_pct, 'uint8_t' );

% right step pulse width (ms)
Mem_size = Mem_size + printCppArray( fid, 'step_rst_pw' , stim_patterns.step_rst_pw, 'uint8_t' );

% right off percentage gait cycle
%GCro

% right off pulse width (ms)
%PWro

% left down percentage gait cycle
%GCld

% left down pulse width (ms)
%PWld

% stand percentage
Mem_size = Mem_size + printCppArray( fid, 'stand_pct' , stim_patterns.stand_pct, 'uint8_t' );

% stand pulse width (ms)
Mem_size = Mem_size + printCppArray( fid, 'stand_pw' , stim_patterns.stand_pw, 'uint8_t' );

% sit percentage
Mem_size = Mem_size + printCppArray( fid, 'sit_pct' , stim_patterns.sit_pct, 'uint8_t' );

% sit pulse width (ms)
Mem_size = Mem_size + printCppArray( fid, 'sit_pw' , stim_patterns.sit_pw, 'uint8_t' );

% print file ending
fprintf(fid, '// File End\r\n\r\n');

% close file
fclose(fid);
disp('-------------------------------------------------------------');
disp('Stim Pattern C Head File saved.');
str = ['    - Use EEPROM memory total size = ',num2str(Mem_size(1)),'KB.'];
disp(str);
disp('Note:');
disp('    - Arduino Uno:    1KB EEPROM storage.');
disp('    - Arduino Mega:   4KB EEPROM storage.');

