function [ data_len ] = printCppArray( file_id, var_name, data_src, data_type )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% print array header
fprintf(file_id, data_type);
fprintf(file_id, ' ');
fprintf(file_id, var_name);
fprintf(file_id, ' = [');

% get size of the data set
data_len = size(data_src);

% print C array
for i=1:data_len(1)-1
    % element 1 to n-1
    fprintf(file_id, '%d, ', data_src(i));
end

% print the last element in the array
fprintf(file_id, '%d];\r\n\r\n', data_src(data_len(1)));

end

