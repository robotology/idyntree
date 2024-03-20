classdef VectorDynSizeVector < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = pop(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(96, self, varargin{:});
    end
    function varargout = brace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(97, self, varargin{:});
    end
    function varargout = setbrace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(98, self, varargin{:});
    end
    function varargout = append(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(99, self, varargin{:});
    end
    function varargout = empty(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(100, self, varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(101, self, varargin{:});
    end
    function varargout = swap(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(102, self, varargin{:});
    end
    function varargout = begin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(103, self, varargin{:});
    end
    function varargout = end(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(104, self, varargin{:});
    end
    function varargout = rbegin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(105, self, varargin{:});
    end
    function varargout = rend(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(106, self, varargin{:});
    end
    function varargout = clear(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(107, self, varargin{:});
    end
    function varargout = get_allocator(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(108, self, varargin{:});
    end
    function varargout = pop_back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(109, self, varargin{:});
    end
    function varargout = erase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(110, self, varargin{:});
    end
    function self = VectorDynSizeVector(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(111, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = push_back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(112, self, varargin{:});
    end
    function varargout = front(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(113, self, varargin{:});
    end
    function varargout = back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(114, self, varargin{:});
    end
    function varargout = assign(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(115, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(116, self, varargin{:});
    end
    function varargout = insert(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(117, self, varargin{:});
    end
    function varargout = reserve(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(118, self, varargin{:});
    end
    function varargout = capacity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(119, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(120, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
