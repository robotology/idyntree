classdef IVectorsVisualization < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1856, self);
        self.SwigClear();
      end
    end
    function varargout = addVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1857, self, varargin{:});
    end
    function varargout = getNrOfVectors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1858, self, varargin{:});
    end
    function varargout = getVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1859, self, varargin{:});
    end
    function varargout = updateVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1860, self, varargin{:});
    end
    function varargout = setVectorColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1861, self, varargin{:});
    end
    function varargout = setVectorsAspect(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1862, self, varargin{:});
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
