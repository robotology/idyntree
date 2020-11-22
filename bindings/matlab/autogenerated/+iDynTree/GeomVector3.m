classdef GeomVector3 < iDynTree.Vector3
  methods
    function self = GeomVector3(varargin)
      self@iDynTree.Vector3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(428, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(429, self, varargin{:});
    end
    function varargout = compose(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(430, self, varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(431, self, varargin{:});
    end
    function varargout = dot(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(432, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(433, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(434, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(435, self, varargin{:});
    end
    function varargout = exp(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(436, self, varargin{:});
    end
    function varargout = cross(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(437, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(438, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
