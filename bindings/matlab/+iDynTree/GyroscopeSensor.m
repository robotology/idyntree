classdef GyroscopeSensor < iDynTree.Sensor
  methods
    function self = GyroscopeSensor(varargin)
      self@iDynTree.Sensor(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(803, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(804, self);
        self.swigPtr=[];
      end
    end
    function varargout = setName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(805, self, varargin{:});
    end
    function varargout = setLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(806, self, varargin{:});
    end
    function varargout = setParent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(807, self, varargin{:});
    end
    function varargout = setParentIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(808, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(809, self, varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(810, self, varargin{:});
    end
    function varargout = getParent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(811, self, varargin{:});
    end
    function varargout = getParentIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(812, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(813, self, varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(814, self, varargin{:});
    end
    function varargout = getLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(815, self, varargin{:});
    end
    function varargout = predictMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(816, self, varargin{:});
    end
  end
  methods(Static)
  end
end
