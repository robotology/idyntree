classdef MatlabSwigIterator < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(5, self);
        self.SwigClear();
      end
    end
    function varargout = value(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(6, self, varargin{:});
    end
    function varargout = incr(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(7, self, varargin{:});
    end
    function varargout = decr(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(8, self, varargin{:});
    end
    function varargout = distance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(9, self, varargin{:});
    end
    function varargout = equal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(10, self, varargin{:});
    end
    function varargout = copy(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(11, self, varargin{:});
    end
    function varargout = next(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(12, self, varargin{:});
    end
    function varargout = previous(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(13, self, varargin{:});
    end
    function varargout = advance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(14, self, varargin{:});
    end
    function varargout = eq(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(15, self, varargin{:});
    end
    function varargout = ne(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(16, self, varargin{:});
    end
    function varargout = TODOincr(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(17, self, varargin{:});
    end
    function varargout = TODOdecr(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(18, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(19, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(20, self, varargin{:});
    end
    function self = MatlabSwigIterator(varargin)
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
