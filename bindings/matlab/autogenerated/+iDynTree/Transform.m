classdef Transform < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Transform(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(794, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = fromHomogeneousTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(795, self, varargin{:});
    end
    function varargout = getSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(796, self, varargin{:});
    end
    function varargout = getRotation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(797, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(798, self, varargin{:});
    end
    function varargout = setRotation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(799, self, varargin{:});
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(800, self, varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(803, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(804, self, varargin{:});
    end
    function varargout = asHomogeneousTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(806, self, varargin{:});
    end
    function varargout = asAdjointTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(807, self, varargin{:});
    end
    function varargout = asAdjointTransformWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(808, self, varargin{:});
    end
    function varargout = log(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(809, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(810, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(811, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(812, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(801, varargin{:});
    end
    function varargout = inverse2(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(802, varargin{:});
    end
    function varargout = Identity(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(805, varargin{:});
    end
  end
end
