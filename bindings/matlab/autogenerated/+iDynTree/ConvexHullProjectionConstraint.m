classdef ConvexHullProjectionConstraint < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = setActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2055, self, varargin{:});
    end
    function varargout = isActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2056, self, varargin{:});
    end
    function varargout = getNrOfConstraints(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2057, self, varargin{:});
    end
    function varargout = projectedConvexHull(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2058, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2059, self, varargin{1});
      end
    end
    function varargout = A(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2060, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2061, self, varargin{1});
      end
    end
    function varargout = b(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2062, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2063, self, varargin{1});
      end
    end
    function varargout = P(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2064, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2065, self, varargin{1});
      end
    end
    function varargout = Pdirection(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2066, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2067, self, varargin{1});
      end
    end
    function varargout = AtimesP(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2068, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2069, self, varargin{1});
      end
    end
    function varargout = o(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2070, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2071, self, varargin{1});
      end
    end
    function varargout = buildConvexHull(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2072, self, varargin{:});
    end
    function varargout = supportFrameIndices(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2073, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2074, self, varargin{1});
      end
    end
    function varargout = absoluteFrame_X_supportFrame(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2075, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2076, self, varargin{1});
      end
    end
    function varargout = project(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2077, self, varargin{:});
    end
    function varargout = computeMargin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2078, self, varargin{:});
    end
    function varargout = setProjectionAlongDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2079, self, varargin{:});
    end
    function varargout = projectAlongDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2080, self, varargin{:});
    end
    function self = ConvexHullProjectionConstraint(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(2081, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(2082, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
