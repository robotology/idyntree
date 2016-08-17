classdef JointDOFsDoubleArray < iDynTree.VectorDynSize
  methods
    function self = JointDOFsDoubleArray(varargin)
      self@iDynTree.VectorDynSize(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(921, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(922, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(923, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(924, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
