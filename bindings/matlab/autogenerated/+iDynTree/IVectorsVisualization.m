classdef IVectorsVisualization < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1924, self);
        self.SwigClear();
      end
    end
    function varargout = addVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1925, self, varargin{:});
    end
    function varargout = getNrOfVectors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1926, self, varargin{:});
    end
    function varargout = getVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1927, self, varargin{:});
    end
    function varargout = updateVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1928, self, varargin{:});
    end
    function varargout = setVectorColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1929, self, varargin{:});
    end
    function varargout = setVectorsDefaultColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1930, self, varargin{:});
    end
    function varargout = setVectorsColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1931, self, varargin{:});
    end
    function varargout = setVectorsAspect(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1932, self, varargin{:});
    end
    function varargout = setVisible(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1933, self, varargin{:});
    end
    function varargout = getVectorLabel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1934, self, varargin{:});
    end
    function self = IVectorsVisualization(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
