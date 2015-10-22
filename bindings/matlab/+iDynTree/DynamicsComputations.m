classdef DynamicsComputations < SwigRef
  methods
    function self = DynamicsComputations(varargin)
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
    function varargout = loadRobotModelFromFile(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(755, self, varargin{:});
    end
    function varargout = loadRobotModelFromString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(756, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(757, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(758, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(759, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(760, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(761, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(762, self, varargin{:});
    end
    function varargout = getFloatingBase(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(763, self, varargin{:});
    end
    function varargout = setFloatingBase(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(764, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(765, self, varargin{:});
    end
    function varargout = getWorldBaseTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(766, self, varargin{:});
    end
    function varargout = getBaseTwist(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(767, self, varargin{:});
    end
    function varargout = getJointPos(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(768, self, varargin{:});
    end
    function varargout = getJointVel(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(769, self, varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(770, self, varargin{:});
    end
    function varargout = getFrameName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(771, self, varargin{:});
    end
    function varargout = getWorldTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(772, self, varargin{:});
    end
    function varargout = getRelativeTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(773, self, varargin{:});
    end
    function varargout = getFrameTwist(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(774, self, varargin{:});
    end
    function varargout = getFrameTwistInWorldOrient(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(775, self, varargin{:});
    end
    function varargout = getFrameProperSpatialAcceleration(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(776, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(777, self, varargin{:});
    end
    function varargout = getLinkInertia(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(778, self, varargin{:});
    end
    function varargout = inverseDynamics(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(779, self, varargin{:});
    end
    function varargout = getFrameJacobian(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(780, self, varargin{:});
    end
    function varargout = getDynamicsRegressor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(781, self, varargin{:});
    end
    function varargout = getModelDynamicsParameters(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(782, self, varargin{:});
    end
  end
  methods(Static)
  end
end
