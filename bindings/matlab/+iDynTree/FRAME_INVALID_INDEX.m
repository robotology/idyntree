function varargout = FRAME_INVALID_INDEX(varargin)
  narginchk(0,1)
  if nargin==0
    nargoutchk(0,1)
    varargout{1} = iDynTreeMEX(690);
  else
    nargoutchk(0,0)
    iDynTreeMEX(691,varargin{1});
  end
end
