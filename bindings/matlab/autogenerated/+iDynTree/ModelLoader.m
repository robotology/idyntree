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
        tmp = iDynTreeMEX(1357, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = loadModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1358, self, varargin{:});
    end
    function varargout = loadModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1359, self, varargin{:});
    end
    function varargout = loadReducedModelFromFullModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1360, self, varargin{:});
    end
    function varargout = loadReducedModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1361, self, varargin{:});
    end
    function varargout = loadReducedModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1362, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1363, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1364, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1365, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1366, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
