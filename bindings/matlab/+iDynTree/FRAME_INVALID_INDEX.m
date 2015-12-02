function varargout = FRAME_INVALID_INDEX(varargin)
  narginchk(0,1)
  if nargin==0
    nargoutchk(0,1)
    varargout{1} = iDynTreeMATLAB_wrap(545);
  else
    nargoutchk(0,0)
    iDynTreeMATLAB_wrap(546,varargin{1});
  end
end
