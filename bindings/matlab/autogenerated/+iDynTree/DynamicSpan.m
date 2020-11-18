classdef DynamicSpan < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = DynamicSpan(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(699, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(700, self);
        self.SwigClear();
      end
    end
    function varargout = first(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(701, self, varargin{:});
    end
    function varargout = last(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(702, self, varargin{:});
    end
    function varargout = subspan(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(703, self, varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(704, self, varargin{:});
    end
    function varargout = size_bytes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(705, self, varargin{:});
    end
    function varargout = empty(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(706, self, varargin{:});
    end
    function varargout = brace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(707, self, varargin{:});
    end
    function varargout = getVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(708, self, varargin{:});
    end
    function varargout = setVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(709, self, varargin{:});
    end
    function varargout = at(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(710, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(711, self, varargin{:});
    end
    function varargout = begin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(712, self, varargin{:});
    end
    function varargout = end(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(713, self, varargin{:});
    end
    function varargout = cbegin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(714, self, varargin{:});
    end
    function varargout = cend(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(715, self, varargin{:});
    end
    function varargout = rbegin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(716, self, varargin{:});
    end
    function varargout = rend(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(717, self, varargin{:});
    end
    function varargout = crbegin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(718, self, varargin{:});
    end
    function varargout = crend(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(719, self, varargin{:});
    end
  end
  methods(Static)
    function v = extent()
      v = iDynTreeMEX(698);
    end
  end
end
