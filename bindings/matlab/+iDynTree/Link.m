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
        tmp = iDynTreeMEX(734, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = inertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(735, self, varargin{:});
    end
    function varargout = setInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(736, self, varargin{:});
    end
    function varargout = getInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(737, self, varargin{:});
    end
    function varargout = setIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(738, self, varargin{:});
    end
    function varargout = getIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(739, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(740, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
