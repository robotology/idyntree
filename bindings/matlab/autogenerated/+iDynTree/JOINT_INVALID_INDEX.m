function varargout = JOINT_INVALID_INDEX(varargin)
  narginchk(0,1)
  if nargin==0
    nargoutchk(0,1)
    varargout{1} = iDynTreeMEX(731);
  else
    nargoutchk(0,0)
    iDynTreeMEX(732,varargin{1});
  end
end
