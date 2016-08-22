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
        tmp = iDynTreeMEX(1172, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = loadModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1173, self, varargin{:});
    end
    function varargout = loadModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1174, self, varargin{:});
    end
    function varargout = loadReducedModelFromFullModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1175, self, varargin{:});
    end
    function varargout = loadReducedModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1176, self, varargin{:});
    end
    function varargout = loadReducedModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1177, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1178, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1179, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1180, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1181, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
