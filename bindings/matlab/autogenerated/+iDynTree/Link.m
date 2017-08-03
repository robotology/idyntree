classdef Link < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Link(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(852, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = inertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(853, self, varargin{:});
    end
    function varargout = setInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(854, self, varargin{:});
    end
    function varargout = getInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(855, self, varargin{:});
    end
    function varargout = setIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(856, self, varargin{:});
    end
    function varargout = getIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(857, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(858, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
