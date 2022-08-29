classdef DHChain < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = setNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1278, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1279, self, varargin{:});
    end
    function varargout = setH0(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1280, self, varargin{:});
    end
    function varargout = getH0(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1281, self, varargin{:});
    end
    function varargout = setHN(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1282, self, varargin{:});
    end
    function varargout = getHN(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1283, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1284, self, varargin{:});
    end
    function varargout = getDOFName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1285, self, varargin{:});
    end
    function varargout = setDOFName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1286, self, varargin{:});
    end
    function varargout = toModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1287, self, varargin{:});
    end
    function varargout = fromModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1288, self, varargin{:});
    end
    function self = DHChain(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1289, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1290, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
