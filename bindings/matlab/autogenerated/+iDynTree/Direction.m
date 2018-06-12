classdef Direction < iDynTree.Vector3
  methods
    function self = Direction(varargin)
      self@iDynTree.Vector3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(618, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = Normalize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(619, self, varargin{:});
    end
    function varargout = isParallel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(620, self, varargin{:});
    end
    function varargout = isPerpendicular(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(621, self, varargin{:});
    end
    function varargout = reverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(622, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(623, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(624, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(626, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = Default(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(625, varargin{:});
    end
  end
end
