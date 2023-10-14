classdef GeomVector3 < iDynTree.Vector3
  methods
    function self = GeomVector3(varargin)
      self@iDynTree.Vector3(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(494, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(495, self, varargin{:});
    end
    function varargout = compose(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(496, self, varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(497, self, varargin{:});
    end
    function varargout = dot(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(498, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(499, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(500, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(501, self, varargin{:});
    end
    function varargout = exp(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(502, self, varargin{:});
    end
    function varargout = cross(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(503, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(504, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
