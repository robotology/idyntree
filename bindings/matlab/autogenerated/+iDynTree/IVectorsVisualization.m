classdef IVectorsVisualization < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(2012, self);
        self.SwigClear();
      end
    end
    function varargout = addVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2013, self, varargin{:});
    end
    function varargout = getNrOfVectors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2014, self, varargin{:});
    end
    function varargout = getVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2015, self, varargin{:});
    end
    function varargout = updateVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2016, self, varargin{:});
    end
    function varargout = setVectorColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2017, self, varargin{:});
    end
    function varargout = setVectorsDefaultColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2018, self, varargin{:});
    end
    function varargout = setVectorsColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2019, self, varargin{:});
    end
    function varargout = setVectorsAspect(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2020, self, varargin{:});
    end
    function varargout = setVisible(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2021, self, varargin{:});
    end
    function varargout = getVectorLabel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2022, self, varargin{:});
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
