classdef Sensor < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(735, self);
        self.swigPtr=[];
      end
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(736, self, varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(737, self, varargin{:});
    end
    function varargout = getParent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(738, self, varargin{:});
    end
    function varargout = getParentIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(739, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(740, self, varargin{:});
    end
    function varargout = setName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(741, self, varargin{:});
    end
    function varargout = setParent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(742, self, varargin{:});
    end
    function varargout = setParentIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(743, self, varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(744, self, varargin{:});
    end
    function self = Sensor(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
