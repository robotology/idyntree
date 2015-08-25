function varargout = JOINT_INVALID_NAME(varargin)
  narginchk(0,1)
  if nargin==0
    nargoutchk(0,1)
    varargout{1} = iDynTreeMATLAB_wrap(338);
  else
    nargoutchk(0,0)
    iDynTreeMATLAB_wrap(339,varargin{1});
  end
end
