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
        tmp = iDynTreeMEX(685, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = getDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(686, self, varargin{:});
    end
    function varargout = getOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(687, self, varargin{:});
    end
    function varargout = setDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(688, self, varargin{:});
    end
    function varargout = setOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(689, self, varargin{:});
    end
    function varargout = getRotationTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(690, self, varargin{:});
    end
    function varargout = getRotationTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(691, self, varargin{:});
    end
    function varargout = getRotationTwist(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(692, self, varargin{:});
    end
    function varargout = getRotationSpatialAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(693, self, varargin{:});
    end
    function varargout = getTranslationTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(694, self, varargin{:});
    end
    function varargout = getTranslationTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(695, self, varargin{:});
    end
    function varargout = getTranslationTwist(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(696, self, varargin{:});
    end
    function varargout = getTranslationSpatialAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(697, self, varargin{:});
    end
    function varargout = isParallel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(698, self, varargin{:});
    end
    function varargout = reverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(699, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(700, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(701, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(702, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
