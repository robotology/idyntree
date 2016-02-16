classdef Axis < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Axis(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(449, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = getDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(450, self, varargin{:});
    end
    function varargout = getOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(451, self, varargin{:});
    end
    function varargout = setDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(452, self, varargin{:});
    end
    function varargout = setOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(453, self, varargin{:});
    end
    function varargout = getRotationTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(454, self, varargin{:});
    end
    function varargout = getRotationTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(455, self, varargin{:});
    end
    function varargout = getRotationTwist(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(456, self, varargin{:});
    end
    function varargout = getRotationSpatialAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(457, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(458, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(459, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(460, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
