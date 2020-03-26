function varargout = LINK_INVALID_INDEX(varargin)
  narginchk(0,1)
  if nargin==0
    nargoutchk(0,1)
    varargout{1} = iDynTreeMEX(873);
  else
    nargoutchk(0,0)
    iDynTreeMEX(874,varargin{1});
  end
end
