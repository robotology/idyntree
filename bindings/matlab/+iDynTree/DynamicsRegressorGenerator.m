classdef DynamicsRegressorGenerator < SwigRef
  methods
    function self = DynamicsRegressorGenerator(varargin)
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
    function varargout = loadRobotAndSensorsModelFromFile(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(755, self, varargin{:});
    end
    function varargout = loadRobotAndSensorsModelFromString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(756, self, varargin{:});
    end
    function varargout = loadRegressorStructureFromFile(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(757, self, varargin{:});
    end
    function varargout = loadRegressorStructureFromString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(758, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(759, self, varargin{:});
    end
    function varargout = getNrOfParameters(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(760, self, varargin{:});
    end
    function varargout = getNrOfOutputs(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(761, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(762, self, varargin{:});
    end
    function varargout = getDescriptionOfParameter(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(763, self, varargin{:});
    end
    function varargout = getDescriptionOfParameters(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(764, self, varargin{:});
    end
    function varargout = getDescriptionOfOutput(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(765, self, varargin{:});
    end
    function varargout = getDescriptionOfOutputs(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(766, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(767, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(768, self, varargin{:});
    end
    function varargout = getBaseLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(769, self, varargin{:});
    end
    function varargout = getSensorsModel(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(770, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(771, self, varargin{:});
    end
    function varargout = getSensorsMeasurements(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(772, self, varargin{:});
    end
    function varargout = computeRegressor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(773, self, varargin{:});
    end
    function varargout = getModelParameters(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(774, self, varargin{:});
    end
    function varargout = computeFloatingBaseIdentifiableSubspace(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(775, self, varargin{:});
    end
    function varargout = computeFixedBaseIdentifiableSubspace(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(776, self, varargin{:});
    end
  end
  methods(Static)
  end
end
