classdef BerdySensor < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = type(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1664, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1665, self, varargin{1});
      end
    end
    function varargout = id(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1666, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1667, self, varargin{1});
      end
    end
    function varargout = range(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1668, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1669, self, varargin{1});
      end
    end
    function varargout = eq(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1670, self, varargin{:});
    end
    function varargout = lt(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1671, self, varargin{:});
    end
    function self = BerdySensor(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1672, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1673, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
