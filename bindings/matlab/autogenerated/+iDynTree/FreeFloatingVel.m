classdef FreeFloatingVel < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = FreeFloatingVel(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
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
    function varargout = baseVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1264, self, varargin{:});
    end
    function varargout = jointVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1265, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1266, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1267, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
