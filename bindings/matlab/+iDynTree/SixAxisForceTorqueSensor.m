classdef SixAxisForceTorqueSensor < iDynTree.Sensor
  methods
    function self = SixAxisForceTorqueSensor(varargin)
      self@iDynTree.Sensor('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(753, varargin{:});
        tmp = iDynTreeMATLAB_wrap(753, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(754, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = setName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(755, self, varargin{:});
    end
    function varargout = setFirstLinkSensorTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(756, self, varargin{:});
    end
    function varargout = setSecondLinkSensorTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(757, self, varargin{:});
    end
    function varargout = getFirstLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(758, self, varargin{:});
    end
    function varargout = getSecondLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(759, self, varargin{:});
    end
    function varargout = setFirstLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(760, self, varargin{:});
    end
    function varargout = setSecondLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(761, self, varargin{:});
    end
    function varargout = getFirstLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(762, self, varargin{:});
    end
    function varargout = getSecondLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(763, self, varargin{:});
    end
    function varargout = setParent(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(764, self, varargin{:});
    end
    function varargout = setParentIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(765, self, varargin{:});
    end
    function varargout = setAppliedWrenchLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(766, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(767, self, varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(768, self, varargin{:});
    end
    function varargout = getParent(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(769, self, varargin{:});
    end
    function varargout = getParentIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(770, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(771, self, varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(772, self, varargin{:});
    end
    function varargout = getAppliedWrenchLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(773, self, varargin{:});
    end
    function varargout = isLinkAttachedToSensor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(774, self, varargin{:});
    end
    function varargout = getLinkSensorTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(775, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(776, self, varargin{:});
    end
  end
  methods(Static)
  end
end
