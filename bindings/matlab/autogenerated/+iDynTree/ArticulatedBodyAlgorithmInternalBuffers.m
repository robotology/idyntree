classdef ArticulatedBodyAlgorithmInternalBuffers < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = ArticulatedBodyAlgorithmInternalBuffers(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1262, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1263, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1264, self, varargin{:});
    end
    function varargout = S(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1265, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1266, self, varargin{1});
      end
    end
    function varargout = U(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1267, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1268, self, varargin{1});
      end
    end
    function varargout = D(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1269, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1270, self, varargin{1});
      end
    end
    function varargout = u(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1271, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1272, self, varargin{1});
      end
    end
    function varargout = linksVel(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1273, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1274, self, varargin{1});
      end
    end
    function varargout = linksBiasAcceleration(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1275, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1276, self, varargin{1});
      end
    end
    function varargout = linksAccelerations(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1277, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1278, self, varargin{1});
      end
    end
    function varargout = linkABIs(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1279, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1280, self, varargin{1});
      end
    end
    function varargout = linksBiasWrench(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1281, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1282, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1283, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
