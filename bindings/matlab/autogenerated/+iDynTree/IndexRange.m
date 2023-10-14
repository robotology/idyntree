classdef IndexRange < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = offset(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(198, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(199, self, varargin{1});
      end
    end
    function varargout = size(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(200, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(201, self, varargin{1});
      end
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(202, self, varargin{:});
    end
    function self = IndexRange(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(204, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(205, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = InvalidRange(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(203, varargin{:});
    end
  end
end
