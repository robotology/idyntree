classdef ConvexHullProjectionConstraint < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = setActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2011, self, varargin{:});
    end
    function varargout = isActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2012, self, varargin{:});
    end
    function varargout = getNrOfConstraints(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2013, self, varargin{:});
    end
    function varargout = projectedConvexHull(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2014, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2015, self, varargin{1});
      end
    end
    function varargout = A(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2016, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2017, self, varargin{1});
      end
    end
    function varargout = b(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2018, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2019, self, varargin{1});
      end
    end
    function varargout = P(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2020, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2021, self, varargin{1});
      end
    end
    function varargout = Pdirection(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2022, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2023, self, varargin{1});
      end
    end
    function varargout = AtimesP(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2024, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2025, self, varargin{1});
      end
    end
    function varargout = o(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2026, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2027, self, varargin{1});
      end
    end
    function varargout = buildConvexHull(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2028, self, varargin{:});
    end
    function varargout = supportFrameIndices(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2029, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2030, self, varargin{1});
      end
    end
    function varargout = absoluteFrame_X_supportFrame(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2031, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2032, self, varargin{1});
      end
    end
    function varargout = project(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2033, self, varargin{:});
    end
    function varargout = computeMargin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2034, self, varargin{:});
    end
    function varargout = setProjectionAlongDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2035, self, varargin{:});
    end
    function varargout = projectAlongDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2036, self, varargin{:});
    end
    function self = ConvexHullProjectionConstraint(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(2037, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(2038, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
