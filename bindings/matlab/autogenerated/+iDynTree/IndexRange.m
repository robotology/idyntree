classdef IndexRange < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = offset(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(123, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(124, self, varargin{1});
      end
    end
    function varargout = size(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(125, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(126, self, varargin{1});
      end
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(127, self, varargin{:});
    end
    function self = IndexRange(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(129, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(130, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = InvalidRange(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(128, varargin{:});
    end
  end
end
