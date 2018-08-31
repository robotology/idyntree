classdef IModelVisualization < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1751, self);
        self.SwigClear();
      end
    end
    function varargout = setPositions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1752, self, varargin{:});
    end
    function varargout = setLinkPositions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1753, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1754, self, varargin{:});
    end
    function varargout = getInstanceName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1755, self, varargin{:});
    end
    function varargout = setModelVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1756, self, varargin{:});
    end
    function varargout = setModelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1757, self, varargin{:});
    end
    function varargout = resetModelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1758, self, varargin{:});
    end
    function varargout = getLinkNames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1759, self, varargin{:});
    end
    function varargout = setLinkVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1760, self, varargin{:});
    end
    function varargout = getFeatures(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1761, self, varargin{:});
    end
    function varargout = setFeatureVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1762, self, varargin{:});
    end
    function varargout = jets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1763, self, varargin{:});
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
