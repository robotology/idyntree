classdef ClassicalAcc < iDynTree.Vector6
  methods
    function self = ClassicalAcc(varargin)
      self@iDynTree.Vector6(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(600, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(601, self, varargin{:});
    end
    function varargout = fromSpatial(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(603, self, varargin{:});
    end
    function varargout = toSpatial(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(604, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(605, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = Zero(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(602, varargin{:});
    end
  end
end
