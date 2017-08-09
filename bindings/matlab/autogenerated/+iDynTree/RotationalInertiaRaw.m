classdef RotationalInertiaRaw < iDynTree.Matrix3x3
  methods
    function self = RotationalInertiaRaw(varargin)
      self@iDynTree.Matrix3x3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(633, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(635, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = Zero(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(634, varargin{:});
    end
  end
end
