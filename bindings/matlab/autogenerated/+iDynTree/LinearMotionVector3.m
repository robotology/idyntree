classdef LinearMotionVector3 < iDynTree.MotionVector3__LinearMotionVector3
  methods
    function self = LinearMotionVector3(varargin)
      self@iDynTree.MotionVector3__LinearMotionVector3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(558, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(559, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(560, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
