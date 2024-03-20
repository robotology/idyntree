classdef SolidShapesVector < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = pop(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1407, self, varargin{:});
    end
    function varargout = brace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1408, self, varargin{:});
    end
    function varargout = setbrace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1409, self, varargin{:});
    end
    function varargout = append(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1410, self, varargin{:});
    end
    function varargout = empty(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1411, self, varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1412, self, varargin{:});
    end
    function varargout = swap(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1413, self, varargin{:});
    end
    function varargout = begin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1414, self, varargin{:});
    end
    function varargout = end(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1415, self, varargin{:});
    end
    function varargout = rbegin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1416, self, varargin{:});
    end
    function varargout = rend(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1417, self, varargin{:});
    end
    function varargout = clear(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1418, self, varargin{:});
    end
    function varargout = get_allocator(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1419, self, varargin{:});
    end
    function varargout = pop_back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1420, self, varargin{:});
    end
    function varargout = erase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1421, self, varargin{:});
    end
    function self = SolidShapesVector(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1422, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = push_back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1423, self, varargin{:});
    end
    function varargout = front(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1424, self, varargin{:});
    end
    function varargout = back(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1425, self, varargin{:});
    end
    function varargout = assign(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1426, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1427, self, varargin{:});
    end
    function varargout = insert(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1428, self, varargin{:});
    end
    function varargout = reserve(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1429, self, varargin{:});
    end
    function varargout = capacity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1430, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1431, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
