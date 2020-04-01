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
        tmp = iDynTreeMEX(1350, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1351, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1352, self, varargin{:});
    end
    function varargout = S(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1353, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1354, self, varargin{1});
      end
    end
    function varargout = U(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1355, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1356, self, varargin{1});
      end
    end
    function varargout = D(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1357, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1358, self, varargin{1});
      end
    end
    function varargout = u(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1359, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1360, self, varargin{1});
      end
    end
    function varargout = linksVel(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1361, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1362, self, varargin{1});
      end
    end
    function varargout = linksBiasAcceleration(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1363, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1364, self, varargin{1});
      end
    end
    function varargout = linksAccelerations(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1365, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1366, self, varargin{1});
      end
    end
    function varargout = linkABIs(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1367, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1368, self, varargin{1});
      end
    end
    function varargout = linksBiasWrench(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1369, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1370, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1371, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
