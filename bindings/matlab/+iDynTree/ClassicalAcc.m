classdef ClassicalAcc < iDynTree.Vector6
  methods
    function self = ClassicalAcc(varargin)
      self@iDynTree.Vector6(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(403, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(404, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(406, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = Zero(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(405, varargin{:});
    end
  end
end
