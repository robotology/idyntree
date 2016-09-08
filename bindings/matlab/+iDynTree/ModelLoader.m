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
        tmp = iDynTreeMEX(1198, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = loadModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1199, self, varargin{:});
    end
    function varargout = loadModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1200, self, varargin{:});
    end
    function varargout = loadReducedModelFromFullModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1201, self, varargin{:});
    end
    function varargout = loadReducedModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1202, self, varargin{:});
    end
    function varargout = loadReducedModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1203, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1204, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1205, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1206, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1207, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
