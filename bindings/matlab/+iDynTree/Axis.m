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
        tmp = iDynTreeMEX(561, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = getDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(562, self, varargin{:});
    end
    function varargout = getOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(563, self, varargin{:});
    end
    function varargout = setDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(564, self, varargin{:});
    end
    function varargout = setOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(565, self, varargin{:});
    end
    function varargout = getRotationTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(566, self, varargin{:});
    end
    function varargout = getRotationTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(567, self, varargin{:});
    end
    function varargout = getRotationTwist(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(568, self, varargin{:});
    end
    function varargout = getRotationSpatialAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(569, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(570, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(571, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(572, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
