classdef ConvexHullProjectionConstraint < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = setActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1960, self, varargin{:});
    end
    function varargout = isActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1961, self, varargin{:});
    end
    function varargout = getNrOfConstraints(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1962, self, varargin{:});
    end
    function varargout = projectedConvexHull(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1963, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1964, self, varargin{1});
      end
    end
    function varargout = A(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1965, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1966, self, varargin{1});
      end
    end
    function varargout = b(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1967, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1968, self, varargin{1});
      end
    end
    function varargout = P(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1969, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1970, self, varargin{1});
      end
    end
    function varargout = Pdirection(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1971, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1972, self, varargin{1});
      end
    end
    function varargout = AtimesP(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1973, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1974, self, varargin{1});
      end
    end
    function varargout = o(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1975, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1976, self, varargin{1});
      end
    end
    function varargout = buildConvexHull(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1977, self, varargin{:});
    end
    function varargout = supportFrameIndices(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1978, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1979, self, varargin{1});
      end
    end
    function varargout = absoluteFrame_X_supportFrame(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1980, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1981, self, varargin{1});
      end
    end
    function varargout = project(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1982, self, varargin{:});
    end
    function varargout = computeMargin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1983, self, varargin{:});
    end
    function varargout = setProjectionAlongDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1984, self, varargin{:});
    end
    function varargout = projectAlongDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1985, self, varargin{:});
    end
    function self = ConvexHullProjectionConstraint(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1986, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1987, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
