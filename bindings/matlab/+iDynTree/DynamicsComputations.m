classdef DynamicsComputations < SwigRef
  methods
    function self = DynamicsComputations(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(366,'new_DynamicsComputations',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(366,'new_DynamicsComputations',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(367,'delete_DynamicsComputations',self);
        self.swigOwn=false;
      end
    end
    function varargout = loadRobotModelFromFile(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(368,'DynamicsComputations_loadRobotModelFromFile',self,varargin{:});
    end
    function varargout = loadRobotModelFromString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(369,'DynamicsComputations_loadRobotModelFromString',self,varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(370,'DynamicsComputations_isValid',self,varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(371,'DynamicsComputations_getNrOfDegreesOfFreedom',self,varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(372,'DynamicsComputations_getDescriptionOfDegreeOfFreedom',self,varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(373,'DynamicsComputations_getDescriptionOfDegreesOfFreedom',self,varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(374,'DynamicsComputations_getNrOfLinks',self,varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(375,'DynamicsComputations_getNrOfFrames',self,varargin{:});
    end
    function varargout = getBaseLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(376,'DynamicsComputations_getBaseLinkName',self,varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(377,'DynamicsComputations_setRobotState',self,varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(378,'DynamicsComputations_getFrameIndex',self,varargin{:});
    end
    function varargout = getWorldTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(379,'DynamicsComputations_getWorldTransform',self,varargin{:});
    end
    function varargout = getRelativeTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(380,'DynamicsComputations_getRelativeTransform',self,varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(381,'DynamicsComputations_getLinkIndex',self,varargin{:});
    end
    function varargout = getLinkInertia(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(382,'DynamicsComputations_getLinkInertia',self,varargin{:});
    end
    function [v,ok] = swig_fieldsref(self,i)
      v = [];
      ok = false;
      switch i
      end
    end
    function [self,ok] = swig_fieldasgn(self,i,v)
      switch i
      end
    end
  end
  methods(Static)
  end
end
