classdef DynamicsComputations < SwigRef
  methods
    function self = DynamicsComputations(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(794, varargin{:});
        tmp = iDynTreeMATLAB_wrap(794, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(795, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = loadRobotModelFromFile(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(796, self, varargin{:});
    end
    function varargout = loadRobotModelFromString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(797, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(798, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(799, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(800, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(801, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(802, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(803, self, varargin{:});
    end
    function varargout = getFloatingBase(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(804, self, varargin{:});
    end
    function varargout = setFloatingBase(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(805, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(806, self, varargin{:});
    end
    function varargout = getWorldBaseTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(807, self, varargin{:});
    end
    function varargout = getBaseTwist(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(808, self, varargin{:});
    end
    function varargout = getJointPos(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(809, self, varargin{:});
    end
    function varargout = getJointVel(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(810, self, varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(811, self, varargin{:});
    end
    function varargout = getFrameName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(812, self, varargin{:});
    end
    function varargout = getWorldTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(813, self, varargin{:});
    end
    function varargout = getRelativeTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(814, self, varargin{:});
    end
    function varargout = getFrameTwist(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(815, self, varargin{:});
    end
    function varargout = getFrameTwistInWorldOrient(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(816, self, varargin{:});
    end
    function varargout = getFrameProperSpatialAcceleration(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(817, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(818, self, varargin{:});
    end
    function varargout = getLinkInertia(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(819, self, varargin{:});
    end
    function varargout = inverseDynamics(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(820, self, varargin{:});
    end
    function varargout = getFrameJacobian(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(821, self, varargin{:});
    end
    function varargout = getDynamicsRegressor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(822, self, varargin{:});
    end
    function varargout = getModelDynamicsParameters(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(823, self, varargin{:});
    end
  end
  methods(Static)
  end
end
