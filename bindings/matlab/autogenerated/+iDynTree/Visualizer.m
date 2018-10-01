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
        tmp = iDynTreeMEX(1788, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1789, self);
        self.SwigClear();
      end
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1790, self, varargin{:});
    end
    function varargout = getNrOfVisualizedModels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1791, self, varargin{:});
    end
    function varargout = getModelInstanceName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1792, self, varargin{:});
    end
    function varargout = getModelInstanceIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1793, self, varargin{:});
    end
    function varargout = addModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1794, self, varargin{:});
    end
    function varargout = modelViz(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1795, self, varargin{:});
    end
    function varargout = camera(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1796, self, varargin{:});
    end
    function varargout = enviroment(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1797, self, varargin{:});
    end
    function varargout = run(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1798, self, varargin{:});
    end
    function varargout = draw(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1799, self, varargin{:});
    end
    function varargout = drawToFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1800, self, varargin{:});
    end
    function varargout = close(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1801, self, varargin{:});
    end
  end
  methods(Static)
  end
end
