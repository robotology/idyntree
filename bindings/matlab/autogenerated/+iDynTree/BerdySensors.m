classdef BerdySensors < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = pop(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(146, self, varargin{:});
    end
    function varargout = brace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(147, self, varargin{:});
    end
    function varargout = setbrace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(148, self, varargin{:});
    end
    function varargout = append(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(149, self, varargin{:});
    end
    function varargout = empty(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(150, self, varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(151, self, varargin{:});
    end
    function varargout = swap(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(152, self, varargin{:});
    end
    function varargout = begin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(153, self, varargin{:});
    end
    function varargout = end(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(154, self, varargin{:});
    end
    function varargout = rbegin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(155, self, varargin{:});
    end
    function varargout = rend(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(156, self, varargin{:});
    end
    function varargout = clear(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(157, self, varargin{:});
    end
    function varargout = get_allocator(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(158, self, varargin{:});
    end
    function varargout = pop_back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(159, self, varargin{:});
    end
    function varargout = erase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(160, self, varargin{:});
    end
    function self = BerdySensors(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(161, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = push_back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(162, self, varargin{:});
    end
    function varargout = front(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(163, self, varargin{:});
    end
    function varargout = back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(164, self, varargin{:});
    end
    function varargout = assign(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(165, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(166, self, varargin{:});
    end
    function varargout = insert(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(167, self, varargin{:});
    end
    function varargout = reserve(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(168, self, varargin{:});
    end
    function varargout = capacity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(169, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(170, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
