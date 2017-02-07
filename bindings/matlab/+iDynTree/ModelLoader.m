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
        tmp = iDynTreeMEX(1290, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = loadModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1291, self, varargin{:});
    end
    function varargout = loadModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1292, self, varargin{:});
    end
    function varargout = loadReducedModelFromFullModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1293, self, varargin{:});
    end
    function varargout = loadReducedModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1294, self, varargin{:});
    end
    function varargout = loadReducedModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1295, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1296, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1297, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1298, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1299, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
