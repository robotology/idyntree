function varargout = LINK_INVALID_NAME(varargin)
  narginchk(0,1)
  if nargin==0
    nargoutchk(0,1)
    varargout{1} = iDynTreeMATLAB_wrap(521);
  else
    nargoutchk(0,0)
    iDynTreeMATLAB_wrap(522,varargin{1});
  end
end
