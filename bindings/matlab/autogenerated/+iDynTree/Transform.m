classdef Transform < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Transform(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(715, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = fromHomogeneousTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(716, self, varargin{:});
    end
    function varargout = getRotation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(717, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(718, self, varargin{:});
    end
    function varargout = setRotation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(719, self, varargin{:});
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(720, self, varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(723, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(724, self, varargin{:});
    end
    function varargout = asHomogeneousTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(726, self, varargin{:});
    end
    function varargout = asAdjointTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(727, self, varargin{:});
    end
    function varargout = asAdjointTransformWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(728, self, varargin{:});
    end
    function varargout = log(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(729, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(730, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(731, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(732, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(721, varargin{:});
    end
    function varargout = inverse2(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(722, varargin{:});
    end
    function varargout = Identity(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(725, varargin{:});
    end
  end
end
