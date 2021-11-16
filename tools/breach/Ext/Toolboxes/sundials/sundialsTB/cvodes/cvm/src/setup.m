
srcdir   = '.' ;
builddir = '.' ;

mexopts  = '' ;
mexflags = '-O' ;

cc          = 'cc' ;
mpicc       = '' ;
mpi_incdir  = '' ;
mpi_libdir  = '' ;
mpi_libs    = '' ;
mpi_flags   = '' ;

stb_parallel = 'no' ;
stb_os       = 'other' ;

if strcmp(stb_parallel,'yes')
  par = true;
else
  par = false;
end

if strcmp(stb_os,'cygwin') || strcmp(stb_os,'mingw')
  par = false;
end

% Get information on where to find various files
% ----------------------------------------------

here = pwd;

% Top sundials source directory
cd(srcdir);
cd('../../../..');
sun_srcdir = pwd;

% Location of cvm mex sources
cvm_srcdir = fullfile(sun_srcdir,'sundialsTB','cvodes','cvm','src','');

% Location of nvm mex sources
nvm_srcdir = fullfile(sun_srcdir,'sundialsTB','nvector','src','');

% Location of sundials header files
sun_incdir = fullfile(sun_srcdir,'include','');

% Location of sundials source files
sun_srcdir = fullfile(sun_srcdir,'src','');

cd(here);

% Top sundials build directory
cd(builddir);
cd('../../../..');
sun_builddir = pwd;

% Location of cvm mex file
cvm_outdir = fullfile(sun_builddir,'sundialsTB','cvodes','cvm','');

% Location of CVODES library
cvodes_libdir = fullfile(sun_builddir,'src','cvodes','.libs','');

% Location of NVEC_SER library
nvecser_libdir = fullfile(sun_builddir,'src','nvec_ser','.libs','');

% Location of NVEC_PAR library
nvecpar_libdir = fullfile(sun_builddir,'src','nvec_par','.libs','');

cd(here);

% Source files
% ------------

% CVM mex sources
cvm_sources = {
    fullfile(cvm_srcdir,'cvm.c')
    fullfile(cvm_srcdir,'cvmWrap.c')
    fullfile(cvm_srcdir,'cvmOpts.c')
              };


% NVM mex sources
if par
    nvm_sources = {
        fullfile(nvm_srcdir,'nvm_parallel.c')
        fullfile(nvm_srcdir,'nvm_ops.c')
                  };
else
    nvm_sources = {
        fullfile(nvm_srcdir,'nvm_serial.c')
        fullfile(nvm_srcdir,'nvm_ops.c')
                  };
end

sources = '';
for i=1:length(cvm_sources)
    sources = sprintf('%s "%s"',sources,cvm_sources{i});
end
for i=1:length(nvm_sources)
    sources = sprintf('%s "%s"',sources,nvm_sources{i});
end

% Preprocessor flags
% ------------------

includes = sprintf('-I"%s" -I"%s" -I"%s" -I"%s"', cvm_srcdir, nvm_srcdir, sun_incdir, sun_srcdir);
if par && ~isempty(mpi_incdir)
  includes = sprintf('%s -I"%s"', includes, mpi_incdir);
end

% Linker flags
% ------------

if strcmp(stb_os,'cygwin') || strcmp(stb_os,'mingw')
  libraries=sprintf('%s %s',...
                    fullfile(cvodes_libdir,'libsundials_cvodes.a'),...
                    fullfile(nvecser_libdir,'libsundials_nvecserial.a'));        
else
  if par
    libraries = sprintf('-L"%s" -lsundials_cvodes -L"%s" -lsundials_nvecparallel',cvodes_libdir, nvecpar_libdir);
    if ~isempty(mpi_libdir)
      libraries = sprintf('%s -L"%s"',libraries, mpi_libdir);
    end
    if ~isempty(mpi_libs)
      libraries = sprintf('%s %s',libraries, mpi_libs);
    end
  else
    libraries = sprintf('-L"%s" -lsundials_cvodes -L"%s" -lsundials_nvecserial',cvodes_libdir, nvecser_libdir);  
  end
end

% Generate and run MEX command
% ----------------------------

if strcmp(stb_os,'cygwin') || strcmp(stb_os,'mingw')
  cc_cmd = '';
else
  if par
    cc_cmd = sprintf('CC=%s',mpicc);
  else
    cc_cmd = sprintf('CC=%s',cc);
  end
end

mex_cmd = sprintf('mex %s %s %s -v -outdir %s -output cvm %s %s %s', ...
                  mexopts, mexflags, cc_cmd, cvm_outdir, includes, sources, libraries);

try
  eval(mex_cmd);
catch
  exit;
end

exit;
