classdef SixAxisForceTorqueSensor < iDynTree.Sensor
  methods
    function self = SixAxisForceTorqueSensor(varargin)
      self@iDynTree.Sensor('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(705, varargin{:});
        tmp = iDynTreeMATLAB_wrap(705, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(706, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = setName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(707, self, varargin{:});
    end
    function varargout = setFirstLinkSensorTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(708, self, varargin{:});
    end
    function varargout = setSecondLinkSensorTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(709, self, varargin{:});
    end
    function varargout = getFirstLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(710, self, varargin{:});
    end
    function varargout = getSecondLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(711, self, varargin{:});
    end
    function varargout = setFirstLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(712, self, varargin{:});
    end
    function varargout = setSecondLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(713, self, varargin{:});
    end
    function varargout = getFirstLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(714, self, varargin{:});
    end
    function varargout = getSecondLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(715, self, varargin{:});
    end
    function varargout = setParent(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(716, self, varargin{:});
    end
    function varargout = setParentIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(717, self, varargin{:});
    end
    function varargout = setAppliedWrenchLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(718, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(719, self, varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(720, self, varargin{:});
    end
    function varargout = getParent(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(721, self, varargin{:});
    end
    function varargout = getParentIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(722, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(723, self, varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(724, self, varargin{:});
    end
    function varargout = getAppliedWrenchLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(725, self, varargin{:});
    end
    function varargout = isLinkAttachedToSensor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(726, self, varargin{:});
    end
    function varargout = getLinkSensorTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(727, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(728, self, varargin{:});
    end
  end
  methods(Static)
  end
end
