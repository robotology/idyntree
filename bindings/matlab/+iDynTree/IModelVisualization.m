classdef IModelVisualization < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1482, self);
        self.swigPtr=[];
      end
    end
    function varargout = setPositions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1483, self, varargin{:});
    end
    function varargout = setLinkPositions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1484, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1485, self, varargin{:});
    end
    function varargout = getInstanceName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1486, self, varargin{:});
    end
    function varargout = setModelVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1487, self, varargin{:});
    end
    function varargout = setModelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1488, self, varargin{:});
    end
    function varargout = resetModelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1489, self, varargin{:});
    end
    function varargout = getLinkNames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1490, self, varargin{:});
    end
    function varargout = setLinkVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1491, self, varargin{:});
    end
    function varargout = getFeatures(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1492, self, varargin{:});
    end
    function varargout = setFeatureVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1493, self, varargin{:});
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
