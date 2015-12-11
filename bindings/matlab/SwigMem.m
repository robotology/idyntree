function varargout = SwigMem(varargin)
  persistent mem
  mlock
  narginchk(0,1)
  if nargin==0
    nargoutchk(0,1)
    varargout{1} = mem;
  else
    nargoutchk(0,0)
    mem = varargin{1};
  end
end
