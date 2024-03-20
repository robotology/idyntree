classdef IFrameVisualization < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(2023, self);
        self.SwigClear();
      end
    end
    function varargout = addFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2024, self, varargin{:});
    end
    function varargout = setVisible(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2025, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2026, self, varargin{:});
    end
    function varargout = getFrameTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2027, self, varargin{:});
    end
    function varargout = updateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2028, self, varargin{:});
    end
    function varargout = getFrameLabel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2029, self, varargin{:});
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
