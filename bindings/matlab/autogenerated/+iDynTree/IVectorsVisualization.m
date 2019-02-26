classdef IVectorsVisualization < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1888, self);
        self.SwigClear();
      end
    end
    function varargout = addVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1889, self, varargin{:});
    end
    function varargout = getNrOfVectors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1890, self, varargin{:});
    end
    function varargout = getVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1891, self, varargin{:});
    end
    function varargout = updateVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1892, self, varargin{:});
    end
    function varargout = setVectorColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1893, self, varargin{:});
    end
    function varargout = setVectorsAspect(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1894, self, varargin{:});
    end
    function self = IVectorsVisualization(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
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
