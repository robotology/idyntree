classdef Axis < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Axis(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(660, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = getDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(661, self, varargin{:});
    end
    function varargout = getOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(662, self, varargin{:});
    end
    function varargout = setDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(663, self, varargin{:});
    end
    function varargout = setOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(664, self, varargin{:});
    end
    function varargout = getRotationTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(665, self, varargin{:});
    end
    function varargout = getRotationTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(666, self, varargin{:});
    end
    function varargout = getRotationTwist(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(667, self, varargin{:});
    end
    function varargout = getRotationSpatialAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(668, self, varargin{:});
    end
    function varargout = getTranslationTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(669, self, varargin{:});
    end
    function varargout = getTranslationTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(670, self, varargin{:});
    end
    function varargout = getTranslationTwist(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(671, self, varargin{:});
    end
    function varargout = getTranslationSpatialAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(672, self, varargin{:});
    end
    function varargout = isParallel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(673, self, varargin{:});
    end
    function varargout = reverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(674, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(675, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(676, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(677, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
