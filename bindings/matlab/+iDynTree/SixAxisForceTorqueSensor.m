classdef SixAxisForceTorqueSensor < iDynTree.Sensor
  methods
    function self = SixAxisForceTorqueSensor(varargin)
      self@iDynTree.Sensor(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(788, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(789, self);
        self.swigPtr=[];
      end
    end
    function varargout = setName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(790, self, varargin{:});
    end
    function varargout = setFirstLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(791, self, varargin{:});
    end
    function varargout = setSecondLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(792, self, varargin{:});
    end
    function varargout = getFirstLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(793, self, varargin{:});
    end
    function varargout = getSecondLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(794, self, varargin{:});
    end
    function varargout = setFirstLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(795, self, varargin{:});
    end
    function varargout = setSecondLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(796, self, varargin{:});
    end
    function varargout = getFirstLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(797, self, varargin{:});
    end
    function varargout = getSecondLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(798, self, varargin{:});
    end
    function varargout = setParent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(799, self, varargin{:});
    end
    function varargout = setParentIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(800, self, varargin{:});
    end
    function varargout = setAppliedWrenchLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(801, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(802, self, varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(803, self, varargin{:});
    end
    function varargout = getParent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(804, self, varargin{:});
    end
    function varargout = getParentIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(805, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(806, self, varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(807, self, varargin{:});
    end
    function varargout = getAppliedWrenchLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(808, self, varargin{:});
    end
    function varargout = isLinkAttachedToSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(809, self, varargin{:});
    end
    function varargout = getLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(810, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(811, self, varargin{:});
    end
  end
  methods(Static)
  end
end
