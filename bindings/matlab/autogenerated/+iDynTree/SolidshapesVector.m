classdef SolidshapesVector < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = pop(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1290, self, varargin{:});
    end
    function varargout = brace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1291, self, varargin{:});
    end
    function varargout = setbrace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1292, self, varargin{:});
    end
    function varargout = append(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1293, self, varargin{:});
    end
    function varargout = empty(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1294, self, varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1295, self, varargin{:});
    end
    function varargout = swap(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1296, self, varargin{:});
    end
    function varargout = begin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1297, self, varargin{:});
    end
    function varargout = end(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1298, self, varargin{:});
    end
    function varargout = rbegin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1299, self, varargin{:});
    end
    function varargout = rend(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1300, self, varargin{:});
    end
    function varargout = clear(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1301, self, varargin{:});
    end
    function varargout = get_allocator(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1302, self, varargin{:});
    end
    function varargout = pop_back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1303, self, varargin{:});
    end
    function varargout = erase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1304, self, varargin{:});
    end
    function self = SolidshapesVector(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1305, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = push_back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1306, self, varargin{:});
    end
    function varargout = front(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1307, self, varargin{:});
    end
    function varargout = back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1308, self, varargin{:});
    end
    function varargout = assign(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1309, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1310, self, varargin{:});
    end
    function varargout = insert(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1311, self, varargin{:});
    end
    function varargout = reserve(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1312, self, varargin{:});
    end
    function varargout = capacity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1313, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1314, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
