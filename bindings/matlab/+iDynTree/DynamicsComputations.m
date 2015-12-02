classdef DynamicsComputations < SwigRef
  methods
    function self = DynamicsComputations(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(825, varargin{:});
        tmp = iDynTreeMATLAB_wrap(825, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(826, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = loadRobotModelFromFile(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(827, self, varargin{:});
    end
    function varargout = loadRobotModelFromString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(828, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(829, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(830, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(831, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(832, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(833, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(834, self, varargin{:});
    end
    function varargout = getFloatingBase(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(835, self, varargin{:});
    end
    function varargout = setFloatingBase(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(836, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(837, self, varargin{:});
    end
    function varargout = getWorldBaseTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(838, self, varargin{:});
    end
    function varargout = getBaseTwist(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(839, self, varargin{:});
    end
    function varargout = getJointPos(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(840, self, varargin{:});
    end
    function varargout = getJointVel(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(841, self, varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(842, self, varargin{:});
    end
    function varargout = getFrameName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(843, self, varargin{:});
    end
    function varargout = getWorldTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(844, self, varargin{:});
    end
    function varargout = getRelativeTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(845, self, varargin{:});
    end
    function varargout = getFrameTwist(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(846, self, varargin{:});
    end
    function varargout = getFrameTwistInWorldOrient(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(847, self, varargin{:});
    end
    function varargout = getFrameProperSpatialAcceleration(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(848, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(849, self, varargin{:});
    end
    function varargout = getLinkInertia(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(850, self, varargin{:});
    end
    function varargout = inverseDynamics(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(851, self, varargin{:});
    end
    function varargout = getFrameJacobian(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(852, self, varargin{:});
    end
    function varargout = getDynamicsRegressor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(853, self, varargin{:});
    end
    function varargout = getModelDynamicsParameters(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(854, self, varargin{:});
    end
  end
  methods(Static)
  end
end
