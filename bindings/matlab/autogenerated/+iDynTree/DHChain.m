classdef DHChain < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = setNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1263, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1264, self, varargin{:});
    end
    function varargout = setH0(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1265, self, varargin{:});
    end
    function varargout = getH0(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1266, self, varargin{:});
    end
    function varargout = setHN(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1267, self, varargin{:});
    end
    function varargout = getHN(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1268, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1269, self, varargin{:});
    end
    function varargout = getDOFName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1270, self, varargin{:});
    end
    function varargout = setDOFName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1271, self, varargin{:});
    end
    function varargout = toModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1272, self, varargin{:});
    end
    function varargout = fromModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1273, self, varargin{:});
    end
    function self = DHChain(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1274, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1275, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
