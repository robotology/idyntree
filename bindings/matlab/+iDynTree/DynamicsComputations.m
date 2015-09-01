classdef DynamicsComputations < SwigRef
  methods
    function self = DynamicsComputations(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(777, varargin{:});
        tmp = iDynTreeMATLAB_wrap(777, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(778, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = loadRobotModelFromFile(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(779, self, varargin{:});
    end
    function varargout = loadRobotModelFromString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(780, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(781, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(782, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(783, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(784, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(785, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(786, self, varargin{:});
    end
    function varargout = getFloatingBase(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(787, self, varargin{:});
    end
    function varargout = setFloatingBase(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(788, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(789, self, varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(790, self, varargin{:});
    end
    function varargout = getWorldTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(791, self, varargin{:});
    end
    function varargout = getRelativeTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(792, self, varargin{:});
    end
    function varargout = getFrameTwist(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(793, self, varargin{:});
    end
    function varargout = getFrameProperSpatialAcceleration(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(794, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(795, self, varargin{:});
    end
    function varargout = getLinkInertia(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(796, self, varargin{:});
    end
  end
  methods(Static)
  end
end
