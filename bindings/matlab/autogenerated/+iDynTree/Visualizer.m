classdef Visualizer < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Visualizer(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1956, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1957, self);
        self.SwigClear();
      end
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1958, self, varargin{:});
    end
    function varargout = getNrOfVisualizedModels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1959, self, varargin{:});
    end
    function varargout = getModelInstanceName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1960, self, varargin{:});
    end
    function varargout = getModelInstanceIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1961, self, varargin{:});
    end
    function varargout = addModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1962, self, varargin{:});
    end
    function varargout = modelViz(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1963, self, varargin{:});
    end
    function varargout = camera(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1964, self, varargin{:});
    end
    function varargout = enviroment(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1965, self, varargin{:});
    end
    function varargout = vectors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1966, self, varargin{:});
    end
    function varargout = run(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1967, self, varargin{:});
    end
    function varargout = draw(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1968, self, varargin{:});
    end
    function varargout = drawToFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1969, self, varargin{:});
    end
    function varargout = close(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1970, self, varargin{:});
    end
  end
  methods(Static)
  end
end
