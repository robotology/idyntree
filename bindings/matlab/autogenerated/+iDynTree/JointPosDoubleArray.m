classdef JointPosDoubleArray < iDynTree.VectorDynSize
  methods
    function self = JointPosDoubleArray(varargin)
      self@iDynTree.VectorDynSize(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1116, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1117, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1118, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1119, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
