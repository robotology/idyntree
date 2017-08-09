classdef LinkWrenches < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = LinkWrenches(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(825, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(826, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(827, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(828, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(829, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(830, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(831, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(832, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
