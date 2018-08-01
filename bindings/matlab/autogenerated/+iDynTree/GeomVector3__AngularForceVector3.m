classdef GeomVector3__AngularForceVector3 < iDynTree.Vector3
  methods
    function varargout = semantics(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(467, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(468, self, varargin{1});
      end
    end
    function self = GeomVector3__AngularForceVector3(varargin)
      self@iDynTree.Vector3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(469, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = setSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(470, self, varargin{:});
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(471, self, varargin{:});
    end
    function varargout = dot(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(474, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(475, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(476, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(477, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(478, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(472, varargin{:});
    end
    function varargout = inverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(473, varargin{:});
    end
  end
end
