classdef ArticulatedBodyAlgorithmInternalBuffers < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = ArticulatedBodyAlgorithmInternalBuffers(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(918, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(919, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(920, self, varargin{:});
    end
    function varargout = S(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(921, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(922, self, varargin{1});
      end
    end
    function varargout = U(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(923, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(924, self, varargin{1});
      end
    end
    function varargout = D(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(925, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(926, self, varargin{1});
      end
    end
    function varargout = u(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(927, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(928, self, varargin{1});
      end
    end
    function varargout = linksVel(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(929, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(930, self, varargin{1});
      end
    end
    function varargout = linksBiasAcceleration(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(931, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(932, self, varargin{1});
      end
    end
    function varargout = linksAccelerations(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(933, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(934, self, varargin{1});
      end
    end
    function varargout = linkABIs(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(935, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(936, self, varargin{1});
      end
    end
    function varargout = linksBiasWrench(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(937, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(938, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(939, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
