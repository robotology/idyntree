classdef BerdySensor < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = type(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1507, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1508, self, varargin{1});
      end
    end
    function varargout = id(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1509, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1510, self, varargin{1});
      end
    end
    function varargout = range(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1511, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1512, self, varargin{1});
      end
    end
    function varargout = eq(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1513, self, varargin{:});
    end
    function varargout = lt(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1514, self, varargin{:});
    end
    function self = BerdySensor(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1515, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1516, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
