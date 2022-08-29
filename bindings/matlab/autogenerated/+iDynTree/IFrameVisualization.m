classdef IFrameVisualization < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1935, self);
        self.SwigClear();
      end
    end
    function varargout = addFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1936, self, varargin{:});
    end
    function varargout = setVisible(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1937, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1938, self, varargin{:});
    end
    function varargout = getFrameTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1939, self, varargin{:});
    end
    function varargout = updateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1940, self, varargin{:});
    end
    function varargout = getFrameLabel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1941, self, varargin{:});
    end
    function self = IFrameVisualization(varargin)
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
