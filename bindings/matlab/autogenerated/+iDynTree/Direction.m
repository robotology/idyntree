classdef Direction < iDynTree.Vector3
  methods
    function self = Direction(varargin)
      self@iDynTree.Vector3(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(594, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = Normalize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(595, self, varargin{:});
    end
    function varargout = isParallel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(596, self, varargin{:});
    end
    function varargout = isPerpendicular(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(597, self, varargin{:});
    end
    function varargout = reverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(598, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(599, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(600, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(602, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = Default(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(601, varargin{:});
    end
  end
end
