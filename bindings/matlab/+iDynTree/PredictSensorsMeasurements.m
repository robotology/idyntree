classdef PredictSensorsMeasurements < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = PredictSensorsMeasurements(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(814, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = makePrediction(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(815, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(816, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
