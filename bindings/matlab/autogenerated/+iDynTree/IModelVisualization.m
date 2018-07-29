classdef IModelVisualization < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1738, self);
        self.SwigClear();
      end
    end
    function varargout = setPositions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1739, self, varargin{:});
    end
    function varargout = setLinkPositions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1740, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1741, self, varargin{:});
    end
    function varargout = getInstanceName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1742, self, varargin{:});
    end
    function varargout = setModelVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1743, self, varargin{:});
    end
    function varargout = setModelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1744, self, varargin{:});
    end
    function varargout = resetModelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1745, self, varargin{:});
    end
    function varargout = getLinkNames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1746, self, varargin{:});
    end
    function varargout = setLinkVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1747, self, varargin{:});
    end
    function varargout = getFeatures(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1748, self, varargin{:});
    end
    function varargout = setFeatureVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1749, self, varargin{:});
    end
    function varargout = jets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1750, self, varargin{:});
    end
    function self = IModelVisualization(varargin)
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
