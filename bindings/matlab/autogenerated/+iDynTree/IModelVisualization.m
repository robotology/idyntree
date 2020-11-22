classdef IModelVisualization < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1841, self);
        self.SwigClear();
      end
    end
    function varargout = setPositions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1842, self, varargin{:});
    end
    function varargout = setLinkPositions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1843, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1844, self, varargin{:});
    end
    function varargout = getInstanceName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1845, self, varargin{:});
    end
    function varargout = setModelVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1846, self, varargin{:});
    end
    function varargout = setModelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1847, self, varargin{:});
    end
    function varargout = resetModelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1848, self, varargin{:});
    end
    function varargout = setLinkColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1849, self, varargin{:});
    end
    function varargout = resetLinkColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1850, self, varargin{:});
    end
    function varargout = getLinkNames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1851, self, varargin{:});
    end
    function varargout = setLinkVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1852, self, varargin{:});
    end
    function varargout = getFeatures(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1853, self, varargin{:});
    end
    function varargout = setFeatureVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1854, self, varargin{:});
    end
    function varargout = jets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1855, self, varargin{:});
    end
    function varargout = getWorldModelTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1856, self, varargin{:});
    end
    function varargout = getWorldLinkTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1857, self, varargin{:});
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
