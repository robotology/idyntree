classdef Visualizer < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Visualizer(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(2070, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(2071, self);
        self.SwigClear();
      end
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2072, self, varargin{:});
    end
    function varargout = getNrOfVisualizedModels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2073, self, varargin{:});
    end
    function varargout = getModelInstanceName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2074, self, varargin{:});
    end
    function varargout = getModelInstanceIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2075, self, varargin{:});
    end
    function varargout = addModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2076, self, varargin{:});
    end
    function varargout = modelViz(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2077, self, varargin{:});
    end
    function varargout = camera(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2078, self, varargin{:});
    end
    function varargout = enviroment(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2079, self, varargin{:});
    end
    function varargout = environment(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2080, self, varargin{:});
    end
    function varargout = vectors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2081, self, varargin{:});
    end
    function varargout = frames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2082, self, varargin{:});
    end
    function varargout = textures(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2083, self, varargin{:});
    end
    function varargout = getLabel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2084, self, varargin{:});
    end
    function varargout = width(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2085, self, varargin{:});
    end
    function varargout = height(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2086, self, varargin{:});
    end
    function varargout = run(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2087, self, varargin{:});
    end
    function varargout = draw(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2088, self, varargin{:});
    end
    function varargout = subDraw(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2089, self, varargin{:});
    end
    function varargout = drawToFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2090, self, varargin{:});
    end
    function varargout = close(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2091, self, varargin{:});
    end
    function varargout = isWindowActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2092, self, varargin{:});
    end
    function varargout = setColorPalette(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2093, self, varargin{:});
    end
  end
  methods(Static)
  end
end
