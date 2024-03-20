classdef MatrixDynSizeVector < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = pop(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(71, self, varargin{:});
    end
    function varargout = brace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(72, self, varargin{:});
    end
    function varargout = setbrace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(73, self, varargin{:});
    end
    function varargout = append(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(74, self, varargin{:});
    end
    function varargout = empty(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(75, self, varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(76, self, varargin{:});
    end
    function varargout = swap(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(77, self, varargin{:});
    end
    function varargout = begin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(78, self, varargin{:});
    end
    function varargout = end(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(79, self, varargin{:});
    end
    function varargout = rbegin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(80, self, varargin{:});
    end
    function varargout = rend(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(81, self, varargin{:});
    end
    function varargout = clear(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(82, self, varargin{:});
    end
    function varargout = get_allocator(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(83, self, varargin{:});
    end
    function varargout = pop_back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(84, self, varargin{:});
    end
    function varargout = erase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(85, self, varargin{:});
    end
    function self = MatrixDynSizeVector(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(86, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = push_back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(87, self, varargin{:});
    end
    function varargout = front(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(88, self, varargin{:});
    end
    function varargout = back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(89, self, varargin{:});
    end
    function varargout = assign(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(90, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(91, self, varargin{:});
    end
    function varargout = insert(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(92, self, varargin{:});
    end
    function varargout = reserve(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(93, self, varargin{:});
    end
    function varargout = capacity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(94, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(95, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
