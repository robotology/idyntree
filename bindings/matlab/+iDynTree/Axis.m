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
        tmp = iDynTreeMEX(421, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(422, self);
        self.swigPtr=[];
      end
    end
    function varargout = getDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(423, self, varargin{:});
    end
    function varargout = getOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(424, self, varargin{:});
    end
    function varargout = setDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(425, self, varargin{:});
    end
    function varargout = setOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(426, self, varargin{:});
    end
    function varargout = getRotationTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(427, self, varargin{:});
    end
    function varargout = getRotationTwist(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(428, self, varargin{:});
    end
    function varargout = getRotationSpatialAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(429, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(430, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(431, self, varargin{:});
    end
  end
  methods(Static)
  end
end
