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
        tmp = iDynTreeMEX(723, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(724, self);
        self.swigPtr=[];
      end
    end
    function varargout = getRotationDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(725, self, varargin{:});
    end
    function varargout = getPositionDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(726, self, varargin{:});
    end
    function varargout = setRotationDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(727, self, varargin{:});
    end
    function varargout = setPositionDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(728, self, varargin{:});
    end
    function varargout = asHomogeneousTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(730, self, varargin{:});
    end
    function varargout = asAdjointTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(731, self, varargin{:});
    end
    function varargout = asAdjointTransformWrenchDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(732, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(733, self, varargin{:});
    end
    function varargout = derivativeOfInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(734, self, varargin{:});
    end
    function varargout = transform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(735, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = Zero(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(729, varargin{:});
    end
  end
end
