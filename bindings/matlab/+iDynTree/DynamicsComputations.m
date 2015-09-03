classdef DynamicsComputations < SwigRef
  methods
    function self = DynamicsComputations(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(733, varargin{:});
        tmp = iDynTreeMATLAB_wrap(733, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(734, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = loadRobotModelFromFile(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(735, self, varargin{:});
    end
    function varargout = loadRobotModelFromString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(736, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(737, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(738, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(739, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(740, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(741, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(742, self, varargin{:});
    end
    function varargout = getFloatingBase(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(743, self, varargin{:});
    end
    function varargout = setFloatingBase(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(744, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(745, self, varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(746, self, varargin{:});
    end
    function varargout = getWorldTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(747, self, varargin{:});
    end
    function varargout = getRelativeTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(748, self, varargin{:});
    end
    function varargout = getFrameTwist(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(749, self, varargin{:});
    end
    function varargout = getFrameProperSpatialAcceleration(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(750, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(751, self, varargin{:});
    end
    function varargout = getLinkInertia(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(752, self, varargin{:});
    end
  end
  methods(Static)
  end
end
