classdef SixAxisForceTorqueSensor < iDynTree.Sensor
  methods
    function self = SixAxisForceTorqueSensor(varargin)
      self@iDynTree.Sensor('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(663, varargin{:});
        tmp = iDynTreeMATLAB_wrap(663, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(664, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = setName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(665, self, varargin{:});
    end
    function varargout = setFirstLinkSensorTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(666, self, varargin{:});
    end
    function varargout = setSecondLinkSensorTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(667, self, varargin{:});
    end
    function varargout = getFirstLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(668, self, varargin{:});
    end
    function varargout = getSecondLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(669, self, varargin{:});
    end
    function varargout = setFirstLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(670, self, varargin{:});
    end
    function varargout = setSecondLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(671, self, varargin{:});
    end
    function varargout = getFirstLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(672, self, varargin{:});
    end
    function varargout = getSecondLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(673, self, varargin{:});
    end
    function varargout = setParent(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(674, self, varargin{:});
    end
    function varargout = setParentIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(675, self, varargin{:});
    end
    function varargout = setAppliedWrenchLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(676, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(677, self, varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(678, self, varargin{:});
    end
    function varargout = getParent(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(679, self, varargin{:});
    end
    function varargout = getParentIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(680, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(681, self, varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(682, self, varargin{:});
    end
    function varargout = getAppliedWrenchLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(683, self, varargin{:});
    end
    function varargout = isLinkAttachedToSensor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(684, self, varargin{:});
    end
    function varargout = getLinkSensorTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(685, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(686, self, varargin{:});
    end
  end
  methods(Static)
  end
end
