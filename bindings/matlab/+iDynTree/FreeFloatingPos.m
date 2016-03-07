classdef FreeFloatingPos < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = FreeFloatingPos(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(876, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(877, self, varargin{:});
    end
    function varargout = worldBasePos(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(878, self, varargin{:});
    end
    function varargout = jointPos(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(879, self, varargin{:});
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(880, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(881, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
