classdef ModelLoader < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = ModelLoader(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1402, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = loadModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1403, self, varargin{:});
    end
    function varargout = loadModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1404, self, varargin{:});
    end
    function varargout = loadReducedModelFromFullModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1405, self, varargin{:});
    end
    function varargout = loadReducedModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1406, self, varargin{:});
    end
    function varargout = loadReducedModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1407, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1408, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1409, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1410, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1411, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
