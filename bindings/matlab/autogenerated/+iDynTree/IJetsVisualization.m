classdef IJetsVisualization < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(2055, self);
        self.SwigClear();
      end
    end
    function varargout = setJetsFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2056, self, varargin{:});
    end
    function varargout = getNrOfJets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2057, self, varargin{:});
    end
    function varargout = getJetDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2058, self, varargin{:});
    end
    function varargout = setJetDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2059, self, varargin{:});
    end
    function varargout = setJetColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2060, self, varargin{:});
    end
    function varargout = setJetsDimensions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2061, self, varargin{:});
    end
    function varargout = setJetsIntensity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2062, self, varargin{:});
    end
    function self = IJetsVisualization(varargin)
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
