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
        tmp = iDynTreeMEX(1922, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1923, self);
        self.SwigClear();
      end
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1924, self, varargin{:});
    end
    function varargout = getNrOfVisualizedModels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1925, self, varargin{:});
    end
    function varargout = getModelInstanceName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1926, self, varargin{:});
    end
    function varargout = getModelInstanceIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1927, self, varargin{:});
    end
    function varargout = addModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1928, self, varargin{:});
    end
    function varargout = modelViz(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1929, self, varargin{:});
    end
    function varargout = camera(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1930, self, varargin{:});
    end
    function varargout = enviroment(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1931, self, varargin{:});
    end
    function varargout = vectors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1932, self, varargin{:});
    end
    function varargout = frames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1933, self, varargin{:});
    end
    function varargout = textures(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1934, self, varargin{:});
    end
    function varargout = getLabel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1935, self, varargin{:});
    end
    function varargout = run(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1936, self, varargin{:});
    end
    function varargout = draw(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1937, self, varargin{:});
    end
    function varargout = drawToFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1938, self, varargin{:});
    end
    function varargout = close(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1939, self, varargin{:});
    end
    function varargout = isWindowActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1940, self, varargin{:});
    end
    function varargout = setColorPalette(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1941, self, varargin{:});
    end
  end
  methods(Static)
  end
end
