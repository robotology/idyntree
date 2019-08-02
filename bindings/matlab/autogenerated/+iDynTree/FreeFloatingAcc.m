classdef FreeFloatingAcc < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = FreeFloatingAcc(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1232, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1233, self, varargin{:});
    end
    function varargout = baseAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1234, self, varargin{:});
    end
    function varargout = jointAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1235, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1236, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1237, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
