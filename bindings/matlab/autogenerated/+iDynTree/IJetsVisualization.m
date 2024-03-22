classdef IJetsVisualization < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1994, self);
        self.SwigClear();
      end
    end
    function varargout = setJetsFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1995, self, varargin{:});
    end
    function varargout = getNrOfJets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1996, self, varargin{:});
    end
    function varargout = getJetDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1997, self, varargin{:});
    end
    function varargout = setJetDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1998, self, varargin{:});
    end
    function varargout = setJetColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1999, self, varargin{:});
    end
    function varargout = setJetsDimensions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2000, self, varargin{:});
    end
    function varargout = setJetsIntensity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2001, self, varargin{:});
    end
    function self = IJetsVisualization(varargin)
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
