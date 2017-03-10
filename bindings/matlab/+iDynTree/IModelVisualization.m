classdef IModelVisualization < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1577, self);
        self.swigPtr=[];
      end
    end
    function varargout = setPositions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1578, self, varargin{:});
    end
    function varargout = setLinkPositions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1579, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1580, self, varargin{:});
    end
    function varargout = getInstanceName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1581, self, varargin{:});
    end
    function varargout = setModelVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1582, self, varargin{:});
    end
    function varargout = setModelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1583, self, varargin{:});
    end
    function varargout = resetModelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1584, self, varargin{:});
    end
    function varargout = getLinkNames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1585, self, varargin{:});
    end
    function varargout = setLinkVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1586, self, varargin{:});
    end
    function varargout = getFeatures(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1587, self, varargin{:});
    end
    function varargout = setFeatureVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1588, self, varargin{:});
    end
    function varargout = jets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1589, self, varargin{:});
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
