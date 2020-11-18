classdef IVectorsVisualization < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1834, self);
        self.SwigClear();
      end
    end
    function varargout = addVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1835, self, varargin{:});
    end
    function varargout = getNrOfVectors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1836, self, varargin{:});
    end
    function varargout = getVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1837, self, varargin{:});
    end
    function varargout = updateVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1838, self, varargin{:});
    end
    function varargout = setVectorColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1839, self, varargin{:});
    end
    function varargout = setVectorsAspect(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1840, self, varargin{:});
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
