function [ numFunc, funcs ] = extractFunctions( filename, varargin )
%EXTRACTFUNCTIONS Extract all functions that are within a specified file
%
% This function will extract all the functions present in the file and save
% them inside the approrpiately named file inside the current directory.
%
% The file will first be copied to a new file named 'testFile.m' before
% being extracted. Ensure there are no functions named 'testFile' to
% prevent this file from possibly being overwritten and corrupted during
% the extraction process.
%
% This will only extract functions at the first indentation level (no
% whitespace in front of the function declaration, so it leaves nested
% functions intact.
%
% This function is able to extract the breakpoint data from the file, and
% then transfer the breakpoints to the newly extracted functions. If
% brekpoint extraction is not selected, then the file is created with no
% breakpoints enabled.
%
% 
% Usage:
%   [ numFunc, funcs ] = extractFunctions( filename );
%   [ numFunc, funcs ] = extractFunctions( filename, ExtractBreakpoints );
%
% Inputs:
%   filename - The name of the file to parse
%   ExtractBreakpoints - Flag to extract breakpoints (0=don't extrac,
%                        1=extract). Defaults to not extracting breakpoints.
%
% Outputs:
%   numFunc - The number of functions extracted from the file
%   funcs   - The names of the functions extracted from the file (in a cell
%             array)
% 
%
% Created by: Ian McInerney
% Created on: February 20, 2018
% Version: 1.2
% Last Modified: February 17, 2019
%
% Revision History
%   1.0 - Initial release
%   1.1 - Added breakpoint extraction
%   1.2 - Made read happen from a temp file to prevent file overwriting


%% Parse the optional input
p = inputParser;
addOptional(p, 'ExtractBreakpoints', 0);

parse(p, varargin{:});
eb = p.Results.ExtractBreakpoints;


%% Copy the file
copyfile(filename, 'tempFile.m', 'f');


%% Open the file
fr = fopen('tempFile.m');

if (fr == -1)
    error('Unable to open the file.');
end


%% Initialize some variables for the loop
fw = -1;
numFunc = 0;
funcs = {};
files = {};
lnum = 1;     % The line number
fLine = [1, 1];


%% Loop while not end of file
while ( ~feof(fr) )
    % Read the line and see if it contains a function declaration
    l = fgets(fr);
    isFunc = regexpi(l, '^[^%\s]*function[\s\S]*\([\s\S]*\)');
    
    % The line defines a function
    if (~isempty( isFunc ) )
        
        % Close the current function file
        if (fw ~= -1)
            fLine(numFunc, 2) = lnum-1;
            fclose(fw);
        end
        
        % Increment the counter
        numFunc = numFunc + 1;
        
        % Save the starting line
        fLine(numFunc, 1) = lnum;
        
        % Extract the function name
        fname = regexpi(l, '[\S]*\(', 'match');
        fname = fname{:};
        fname = fname(1:end-1);
        
        % Save the function name for return
        funcs{numFunc} = fname;
        
        % Make the file name
        fname = [fname, '.m'];
        files{numFunc} = fname;
        
        % Open the file
        fw = fopen(fname, 'w');
        
        % If the file was unable to be opened for some reason, throw an
        % error and close the reading file
        if (fw == -1)
            fclose(fr);
            error(['Unable to open file ', fname]);
        end
    end
    
    lnum = lnum + 1;
    
    % Write to the function file if it is open
    if (fw ~= -1)
        fprintf(fw, '%s', l);
    end
end

% Add the last line
fLine(numFunc, 2) = lnum;


%% Close the files that are open
fclose(fr);
if (fw ~= -1)
    fclose(fw);
end


%% Delete the temp file
delete('tempFile.m');


%% Force MATLAB to find the new functions
rehash


%% Clear the debug information for the files that have been extracted
for ( i=1:1:numFunc )
    eval(['dbclear ', files{i}]);
end


%% Extract debug information if requested
if (eb)
    
    % Get the breakpoint information for the original file
    bp = dbstatus(filename);
    numBP = length(bp);
    
    % Iterate through each breakpoint
    for ( i=1:1:numBP )
        % Figure out which function it goes with
        condMet = (bp(i).line >= fLine(:,1)) + (bp(i).line <= fLine(:,2));
        fInd = find( condMet == 2 );
        
        lNum = bp(i).line - fLine(fInd,1) + 1;
        
        % Extract the expression if it is there
        expre = bp(i).expression{:};
        
        % Set the new breakpoint
        if ( isempty(expre) )
            eval(['dbstop in ', files{fInd}, ' at ', num2str(lNum), ';']);
        else
            eval(['dbstop in ', files{fInd}, ' at ', num2str(lNum), ' if ''', expre, ''';']);
        end
        
    end
    
end

end

