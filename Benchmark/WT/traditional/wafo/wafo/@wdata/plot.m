function H1=plot(self,varargin)
%WDATA/PLOT Plot contents of WDATA objects
%
% CALL: H = plot(f,plotflag,x1,x2,x3,sym,method,shading)
%
%  H = handle to the created object
%
%  plot a WDATA object with the fields
%      
%      f.data   = vector/matrix
%      f.args   = vector or cellarray of values in n dimensions (n=1:3) 
%  
% optional fields:   
%      f.labels        = cellarray of label strings          (n=1:3) 
%      f.title         = title string
%      f.contourLevels = contour levels for 2D data
%      f.percentlevels = Percent levels the given contour
%                        lines encloses.  
%     1D:
%      plotflag =  Integer giving plot type, transformation of
%                   data and plot scale defined as: 
%                PlotType  = mod(plotflag,10);
%                TransType = mod(floor(plotflag/10),10);
%                logXscale = mod(floor(plotflag/100),10)>0;
%                logYscale = mod(floor(plotflag/1000),10)>0;
%                logZscale = mod(floor(plotflag/10000),10)>0;  
%          PlotType options are:
%                 1  linear plot of T(x)        (default)
%                 2  stairs plot of T(x)
%                 3  stem plot of T(x)
%                 4  errorbar plot of T(x) (requires nonempty dataCI)%
%                 5  bar plot of T(x)
%                 6  area plot of T(x)
%          where T(x) is the transformed data according to TransType.
%          TransType options are:
%                 0 T(x) = f.data, no transformation
%                 1 T(x) = 1-f.data
%                 2 T(x) = F     = cumtrapz(f.args,f.data)
%                 3 T(x) = 1-F(x) 
%                 4 T(x) = -log(1-F(x)) 
%                 5 T(x) = 10*log10(f.data) 
%
%     2D:
%      plotflag = 1 contour plot (default)
%                 2 mesh
%                 3 surf
%                 4 waterfall
%                 5 pcolor
%                 6 contour3
%     3D: 
%      plotflag = 1 sliceomatic   (default) 
%                 2 sliceomatic (with index x-,y-and z-labels)
%                 3 slice       
%                 4 contour f(X1,X2,X3(x1)), where x1 is an integer
%                 5 contour f(X1(x1),X2,X3), where x1 is an integer
%                 6 contour f(X1,X2(x1),X3), where x1 is an integer
%      x1,x2,x3 = are vectors defining where to slice for 3D data
%                 (default along the axis where f has its maximum)
%      sym      = plot symbol (default '-')  
%      method   = interpolation method for 3D slice 
%                 'linear' (default), 'cubic', or 'nearest'
%      shading  = controls the color shading of SURFACE and PATCH
%                 objects.  'faceted' (default), flat or 'interp'
%
%  Note: - sym,method and shading can be given anywhere after f and in
%          any order.
%        - sliceomatic must be downloaded from 
%          <http://www.mathworks.com/matlabcentral/fileexchange/>
%
% See also   data_1d/plot, data_2d/plot, datastructures, qlevels, cltext

% Note: is only able to handle 1D,2D and 3D plot i.e. ndim=3

%Tested on: Matlab 5.3, 5.2
%History:
% by pab March 2007
% - based on old pdfplot





hold_state = ishold; % remember old hold state
Nff=length(self);
if Nff>1
  cfig=gcf;
  H = zeros(1,Nff);
  for ix=1:Nff
    if hold_state
      newplot
    else
      figure(cfig-1+ix)
    end
    H(ix) = plot(self(ix),varargin{:});
  end
  if nargout>0
    H1 = H;
  end
  return
end

%cax  = newplot; % axes
%cfig = get(cax,'Parent'); %cfig=gcf;


H = plot(self.type,self,varargin{:});
if (nargout>=1)
  H1=H;
end
end

