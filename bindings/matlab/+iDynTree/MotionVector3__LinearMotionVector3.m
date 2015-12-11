classdef MotionVector3__LinearMotionVector3 < iDynTree.GeomVector3__LinearMotionVector3
  methods
    function self = MotionVector3__LinearMotionVector3(varargin)
      self@iDynTree.GeomVector3__LinearMotionVector3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(286, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(287, self);
        self.swigPtr=[];
      end
    end
    function varargout = cross(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(288, self, varargin{:});
    end
  end
  methods(Static)
  end
end
