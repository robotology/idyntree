classdef JointDOFsDoubleArray < iDynTree.VectorDynSize
  methods
    function self = JointDOFsDoubleArray(varargin)
      self@iDynTree.VectorDynSize(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1225, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1226, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1227, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1228, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
