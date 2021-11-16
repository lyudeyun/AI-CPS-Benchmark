function mfilesummary(startpath,recurse,delefirst,includecalllist)

% MFILESUMMARY(STARTPATH,RECURSE,DELEFIRST,INCLUDECALLLIST) creates a summary file ('mfilesummary.doc')
% in the directory of the specified starting path. This file summarizes all mfiles in the search
% path (searching recursively or non-recursively) by showing the syntax (i.e., the text of the first,
% or function-declaring, line), by showing which other mfiles on the search path call the given mfile
% (or, at least, which other mfiles contain the name of the given function), and by showing which other
% mfiles on the search path that the given mfile calls (or, at least, which other mfiles that the given
% mfile contains the name of).
%
% NO ARGUMENTS ARE REQUIRED; gui pop-ups will prompt the user for all necessary information. However, the arguments
%           may be specified instead.
% ARGUMENTS:
% STARTPATH: steers the program toward the search path by selecting any mfile at the top level of the search.
% RECURSE (= 0 or 1): search is recursive, or non-recursive (including or omitting sub-directories).
% DELEFIRST (= 0 or 1): prompts the program to delete existing instance of mfilesummary.doc
%           in the starting path. (If the file exists and is not deleted, new information is appended to the 
%           bottom of the file.
% INCLUDECALLLIST (= 0 or 1): instructs the program to include (or exclude) summaries of calls to and from the
%           summarized mfile.
%
% NOTE: With includecalllist == 1, the run takes a few minutes, but the resulting output file can be quite useful
% as an overall summary of the mfiles on a search path. With includecallist == 0, the runtime is much faster, but
% the summarized information comprises only the text of the first (function-declaring) line.
%
% THIS MFILE CALLS (AND THUS REQUIRES THE PRESENCE OF) MFILEGREP.
%
% Copyright Brett Shoelson, Ph.D.
% Shoelson Consulting
% brett.shoelson@joslin.harvard.edu
% V2: 6/23/01. Extensive modifications include on/off of recursion, handling of different numbers of input arguments.

if ~nargin
	% Get starting path
	[filename,startpath]=uigetfile('*.m','Select any m-file in the desired starting pathname.');
	%Remove trailing '\' character
	if strcmp(startpath(end),'\'),startpath=startpath(1:end-1);end
	% Query for inclusion of subdirectories
	recurse=questdlg('Include subdirectories?','','YES','No','YES');
	if strcmp(recurse,'YES'),recurse=1;else;recurse=0;end
	% Query for deletion of existing mfilesummary.doc file
	delefirst=questdlg('Clear _mfilesummary.doc_ first?','mfilesummary.m','YES','No','YES');
	if strcmp(delefirst,'YES'), delefirst=1;else;delefirst=0;end
	includecalllist = questdlg('Include call list?','','YES','No','YES');
	if strcmp(includecalllist,'YES'),includecalllist = 1;else; includecalllist = 0; end
elseif nargin == 1
	recurse=questdlg('Include subdirectories?','','YES','No','YES');
	if strcmp(recurse,'YES'),recurse=1;else;recurse=0;end
	delefirst=questdlg('Clear _mfilesummary.doc_ first?','mfilesummary.m','YES','No','YES');
	if strcmp(delefirst,'YES'), delefirst=1;else;delefirst=0;end
	includecalllist = questdlg('Include call list?','','YES','No','YES');
	if strcmp(includecalllist,'YES'),includecalllist = 1;else; includecalllist = 0; end
elseif nargin == 2
	delefirst=questdlg('Clear _mfilesummary.doc_ first?','mfilesummary.m','YES','No','YES');
	if strcmp(delefirst,'YES'), delefirst=1;else;delefirst=0;end
	includecalllist = questdlg('Include call list?','','YES','No','YES');
	if strcmp(includecalllist,'YES'),includecalllist = 1;else; includecalllist = 0; end
elseif nargin == 3
	includecalllist = questdlg('Include call list?','','YES','No','YES');
	if strcmp(includecalllist,'YES'),includecalllist = 1;else; includecalllist = 0; end
elseif nargin > 4
	error('Too many input arguments.')
end

% Delete instances of 'codetext.txt' in search path.
if delefirst
	files=filesearch('doc',startpath,recurse);
	for i=1:length(files)
		if ~isempty(findstr(files{i},'mfilesummary.doc'))
			try
				delete(files{i});
			catch
			end
		end
	end
end
startcalllistpath = 'D:\mfiles';

% Compile filenames (calls subfunction filesearch)
files=filesearch('m',startpath,recurse);
cd(startpath);

% Open codetext.txt for appending. Create if necessary.
fid1=fopen([startpath '\mfilesummary.doc'],'at');

marker1='%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%';
marker2='%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%';
line = 0;

% Loop through all mfiles on search path
for i=1:size(files,1)
	filename=files{i};
	slashpos = findstr(filename,'\');
	dotpos = findstr(filename,'.');
	shortname = filename(max(slashpos)+1:max(dotpos)-1);
	fprintf('Summarizing %s text.\n',filename);
	% Open mfile(i) and read contents line by line; write lines to codetext.txt
	fid2=fopen(filename,'rt');
	fprintf(fid1,'\n\n%s\n%s\n%s\n\n',marker1,filename,marker2);
	line=fgets(fid2);
	%line=0;
	% Close mfile(i) and continue
	fclose(fid2);
	if line~=-1
		fprintf(fid1,'SYNTAX:\n%s',line);
		if includecalllist
			a = mfilegrep(shortname,startcalllistpath,1);
			if ~ isempty(a)
				fprintf(fid1,'\nCALLED FROM:\n');
			end
			for j = 1 : length(a)
				fprintf(fid1,'\n%s',num2str(a{j}));
			end
			a = findmcallsfrom(filename,startcalllistpath,1,1,0);
			if ~ isempty(a)
				fprintf(fid1,'\n\nCALLS TO:\n');
			end
			for j = 1 : length(a)
				fprintf(fid1,'\n%s',num2str(a{j}));
			end
		end
	end
end
% Close codetext.txt 
fclose(fid1);
fprintf('\n\nDone. Stored in %s.\n',[startpath '\mfilesummary.doc']);



function files = filesearch(extension,startpath,recurse)
%
%  Search a specified path (recursively or non-recursively) for all instances of files with a specified extension.
%  Results are returned in a cell array ('files').
%
%  FILES = FILESEARCH(EXTENSION,STARTPATH,1) searches for all instances of *.extension in the starting directory OR
%          in any subdirectory beneath the starting directory.
%  FILES = FILESEARCH(EXTENSION,STARTPATH,0) searches for all instances of *.extension in the starting directory ONLY,
%          and excludes files in subdirectories.
%  Examples:
% 
%  files = filesearch('txt','C:\WinNT',1)
%  mfiles = filesearch('m','D:\Mfiles',0)

% Copyright Brett Shoelson, Ph.D., 5/29/2001 
% brett.shoelson@joslin.harvard.edu

if recurse
	files = {};
	%Create string of recursive directories/subdirectories
	paths = genpath(startpath);
	%Find instances of the path separator
	seplocs = findstr(paths,pathsep);
	%Parse paths into individual (vertically catenated) directories
	if ~isempty(seplocs)
		directories = paths(1:seplocs(1)-1);
		for i = 1:length(seplocs)-1
			directories = strvcat(directories,paths(seplocs(i)+1:seplocs(i+1)-1));
		end
		%Search each directory for instances of *.extension, appending to current list
		for i = 1:size(directories,1)
			%Compile located files as vertically catenated strings
			tmp = dir([deblank(directories(i,:)) '\*.' extension]);
			if ~isempty(tmp),tmp = char(tmp.name);end
			%Update files to reflect newly detected files. (Omit trailing blanks.)
			for j = 1:size(tmp,1)
				files{size(files,1)+1,1} = [deblank(directories(i,:)) '\' deblank(tmp(j,:))];
			end
		end
	end
else %Search non-recursively
	files = {};
	tmp = dir([startpath '\*.' extension]);
	if ~isempty(tmp),tmp = char(tmp.name);end
	for j = 1:size(tmp,1)
		files{size(files,1)+1,1} = [startpath '\' deblank(tmp(j,:))];
	end
end


function calllist = findmcallsfrom(checkfile,startpath,recurse,wholewords,casesens)
% MFILELIST = FINDMCALLSTO(SEARCHSTRING, STARTPATH, RECURSE, WHOLEWORDS, CASESENS)
% returns the names of all m-files on the search string called (recursively, or non-recursively) by the file checkfile.m.
%
% The function calls the embedded subfunction 'filesearch' to locate the files.
%
% Requires 1 to 5 input arguments; at a minimum, the user must specify the desired searchstring.
% startpath and recurse (= 0 or 1) may also be inputs; otherwise, a gui interface will prompt the
% user for these values.
% wholewords and casesens (= 0 or 1) may be indicated as well; if not, the default values of 1 and 0,
% respectively, will be used.
%
% EXAMPLES:
%
% FILELIST = FINDMCALLSTO('domct') prompts the user via uicontrols for starting path and inclusion or 
%             exclusion of subdirectories, then searches all mfiles on the indicated path for 'domct'.
% FILELIST = FINDMCALLSTO('domct', 'C:\Mfiles') searches specified directory for m-files; prompts the user for:
%             inclusion or exclusion of subdirectories/\.
% FILELIST = FINDMCALLSTO('domct','C:\Mfiles',1) searches specified directory recursively for m-files.
% FILELIST = FINDMCALLSTO('domct','C:\Mfiles',1, 0, 1) searches specified directory recursively for m-files;
%             in this case, the search returns partial word matches (e.g., 'examp' matches 'example'),
%             and the search is case sensitive.

% Copyright Brett Shoelson, Ph.D.
% Shoelson Consulting
% brett.shoelson@joslin.harvard.edu
% V1: 6/22/01. Based extensively on the new version of collectcode.m.


if ~nargin
	error('Requires at least one argument indicating desired searchstring.');
elseif nargin == 1
	% Get starting path
	[filename,startpath]=uigetfile('*.m','Select any m-file in the desired starting pathname.');
	%Remove trailing '\' character
	if strcmp(startpath(end),'\'),startpath=startpath(1:end-1);end
	% Query for inclusion of subdirectories
	recurse=questdlg('Include subdirectories?','','YES','No','YES');
	if strcmp(recurse,'YES'),recurse=1;else;recurse=0;end
	wholewords = 1; casesens = 0;
elseif nargin == 2
	recurse=questdlg('Include subdirectories?','','YES','No','YES');
	if strcmp(recurse,'YES'),recurse=1;else;recurse=0;end
	wholewords = 1; casesens = 0;
elseif nargin == 3
	wholewords = 1; casesens = 0;
elseif nargin == 4
	casesens = 0;
elseif nargin > 5
	error('Too many input arguments.');
end

% Check for appropriateness of inputs:
if ~ischar(checkfile)
	error('searchstring must be a character string.');
end
if recurse ~= 1 & recurse ~= 0
	error('recurse must be 1 or 0.');
end

if wholewords
	alphachars = ['A' ; 'B' ; 'C' ; 'D' ; 'E' ; 'F' ; 'G' ; 'H' ; 'I' ; 'J' ; 'K' ; 'L' ; ...
			'M' ; 'N' ; 'O' ; 'P' ; 'Q' ; 'R' ; 'S' ; 'T' ; 'U' ; 'V' ; 'W' ; 'X' ; ...
			'Y' ; 'Z'];
	alphachars = [alphachars; lower(alphachars); '1' ; '2' ; '3' ; '4' ; '5' ; '6' ; '7' ; '8' ; '9' ; '0'];
end

currpath = cd;

% Compile filenames (calls subfunction filesearch)
files = filesearch('m', startpath, recurse);
mnames = cell(length(files),1);
for i = 1:length(files)
	[x,name,ext] = fileparts(files{i});
	mnames{i} = name;
end
[mnames,iii,jjj] = unique(mnames);
cd(startpath);

line = 0;
calllist = {};

if ~ casesens
	mnames = lower(mnames);
end

% Open checkfile.m and loop through mnames

fprintf('Searching specified file....\n');

fid3 = fopen(checkfile, 'rt');
if fid3 == -1
	error('Unable to open specified checkfile.')
end

while line ~=-1
	line = fgets(fid3);
	if line ~= -1
		if ~ casesens
			line = lower(line);
		end
		for i = 1:length(mnames)
			searchstring = mnames{i};
			if ~isempty(findstr(line, searchstring))
				% findstr finds the instance of the shorter string in the longer; the following if/then ensures that
				% the function won't return instances of subelements (eg., findstr('y','classify') is non-empty).
				if length(line) >= length(searchstring)
					if wholewords
						wholefound = iswhole(searchstring, line, alphachars);
						if wholefound
							%calllist{size(calllist, 1) + 1, 1} = searchstring;
							calllist{size(calllist, 1) + 1, 1} = files{iii(i)};
							break
						end
					else
						%calllist{size(calllist, 1) + 1, 1} = searchstring;
						calllist{size(calllist, 1) + 1, 1} = files{iii(i)};
						break
					end % if wholewords
				end % if length(line) >= length(searchstring)
			end % if ~isempty(findstr(line, searchstring))
		end %for i = 1:length(mnames)
	end % if line ~=- 1
end % while line ~=- 1
line = 0;
% Close mfile(i) and continue
fclose(fid3);

calllist = unique(calllist);
	
cd(currpath);


function files = filesearch(extension,startpath,recurse)
%
%  Search a specified path (recursively or non-recursively) for all instances of files with a specified extension.
%  Results are returned in a cell array ('files').
%
%  FILES = FILESEARCH(EXTENSION,STARTPATH,1) searches for all instances of *.extension in the starting directory OR
%          in any subdirectory beneath the starting directory.
%  FILES = FILESEARCH(EXTENSION,STARTPATH,0) searches for all instances of *.extension in the starting directory ONLY,
%          and excludes files in subdirectories.
%  Examples:
% 
%  files = filesearch('txt','C:\WinNT',1)
%  mfiles = filesearch('m','D:\Mfiles',0)

% Copyright Brett Shoelson, Ph.D., 5/29/2001 
% brett.shoelson@joslin.harvard.edu

if recurse
	files = {};
	%Create string of recursive directories/subdirectories
	paths = genpath(startpath);
	%Find instances of the path separator
	seplocs = findstr(paths,pathsep);
	%Parse paths into individual (vertically catenated) directories
	if ~isempty(seplocs)
		directories = paths(1:seplocs(1)-1);
		for i = 1:length(seplocs)-1
			directories = strvcat(directories,paths(seplocs(i)+1:seplocs(i+1)-1));
		end
		%Search each directory for instances of *.extension, appending to current list
		for i = 1:size(directories,1)
			%Compile located files as vertically catenated strings
			tmp = dir([deblank(directories(i,:)) '\*.' extension]);
			if ~isempty(tmp),tmp = char(tmp.name);end
			%Update files to reflect newly detected files. (Omit trailing blanks.)
			for j = 1:size(tmp,1)
				files{size(files,1)+1,1} = [deblank(directories(i,:)) '\' deblank(tmp(j,:))];
			end
		end
	end
else %Search non-recursively
	files = {};
	tmp = dir([startpath '\*.' extension]);
	if ~isempty(tmp),tmp = char(tmp.name);end
	for j = 1:size(tmp,1)
		files{size(files,1)+1,1} = [startpath '\' deblank(tmp(j,:))];
	end
end


function wholefound = iswhole(searchstring, teststring, alphachars)
charlocs = findstr(searchstring, teststring);
prevlocs = charlocs - 1;
postlocs = charlocs + length(searchstring);
delprevposns = [find(prevlocs == 0)];
delpostposns = [find(postlocs >= length(teststring))];
prevlocs(delprevposns) = '';
postlocs(delpostposns) = '';
prevmbrs = ismember(teststring(prevlocs)', alphachars);
postmbrs = ismember(teststring(postlocs)', alphachars);
if ~ isempty(delprevposns), prevmbrs = [0; prevmbrs]; end
if ~ isempty(delpostposns), postmbrs = [postmbrs; 0]; end
wholefound = any(~ or (prevmbrs, postmbrs) );
