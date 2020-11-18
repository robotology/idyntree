classdef ClassicalAcc < iDynTree.Vector6
  methods
    function self = ClassicalAcc(varargin)
      self@iDynTree.Vector6(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(522, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(523, self, varargin{:});
    end
    function varargout = fromSpatial(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(525, self, varargin{:});
    end
    function varargout = toSpatial(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(526, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(527, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = Zero(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(524, varargin{:});
    end
  end
end
