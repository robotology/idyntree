classdef TransformDerivative < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = TransformDerivative(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(742, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(743, self);
        self.swigPtr=[];
      end
    end
    function varargout = getRotationDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(744, self, varargin{:});
    end
    function varargout = getPositionDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(745, self, varargin{:});
    end
    function varargout = setRotationDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(746, self, varargin{:});
    end
    function varargout = setPositionDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(747, self, varargin{:});
    end
    function varargout = asHomogeneousTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(749, self, varargin{:});
    end
    function varargout = asAdjointTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(750, self, varargin{:});
    end
    function varargout = asAdjointTransformWrenchDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(751, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(752, self, varargin{:});
    end
    function varargout = derivativeOfInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(753, self, varargin{:});
    end
    function varargout = transform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(754, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = Zero(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(748, varargin{:});
    end
  end
end
