classdef IRawMatrix < iDynTree.IMatrix
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(11, self);
        self.swigPtr=[];
      end
    end
    function varargout = data(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(12, self, varargin{:});
    end
    function self = IRawMatrix(varargin)
      self@iDynTree.IMatrix(SwigRef.Null);
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
