classdef Model < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Model(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1045, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = copy(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1046, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1047, self);
        self.SwigClear();
      end
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1048, self, varargin{:});
    end
    function varargout = getLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1049, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1050, self, varargin{:});
    end
    function varargout = isValidLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1051, self, varargin{:});
    end
    function varargout = getLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1052, self, varargin{:});
    end
    function varargout = addLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1053, self, varargin{:});
    end
    function varargout = getNrOfJoints(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1054, self, varargin{:});
    end
    function varargout = getJointName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1055, self, varargin{:});
    end
    function varargout = getTotalMass(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1056, self, varargin{:});
    end
    function varargout = getJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1057, self, varargin{:});
    end
    function varargout = getJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1058, self, varargin{:});
    end
    function varargout = isValidJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1059, self, varargin{:});
    end
    function varargout = isLinkNameUsed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1060, self, varargin{:});
    end
    function varargout = isJointNameUsed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1061, self, varargin{:});
    end
    function varargout = isFrameNameUsed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1062, self, varargin{:});
    end
    function varargout = addJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1063, self, varargin{:});
    end
    function varargout = addJointAndLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1064, self, varargin{:});
    end
    function varargout = insertLinkToExistingJointAndAddJointForDisplacedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1065, self, varargin{:});
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1066, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1067, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1068, self, varargin{:});
    end
    function varargout = addAdditionalFrameToLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1069, self, varargin{:});
    end
    function varargout = getFrameName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1070, self, varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1071, self, varargin{:});
    end
    function varargout = isValidFrameIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1072, self, varargin{:});
    end
    function varargout = getFrameTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1073, self, varargin{:});
    end
    function varargout = getFrameLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1074, self, varargin{:});
    end
    function varargout = getLinkAdditionalFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1075, self, varargin{:});
    end
    function varargout = getNrOfNeighbors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1076, self, varargin{:});
    end
    function varargout = getNeighbor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1077, self, varargin{:});
    end
    function varargout = setDefaultBaseLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1078, self, varargin{:});
    end
    function varargout = getDefaultBaseLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1079, self, varargin{:});
    end
    function varargout = computeFullTreeTraversal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1080, self, varargin{:});
    end
    function varargout = getInertialParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1081, self, varargin{:});
    end
    function varargout = updateInertialParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1082, self, varargin{:});
    end
    function varargout = visualSolidShapes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1083, self, varargin{:});
    end
    function varargout = collisionSolidShapes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1084, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1085, self, varargin{:});
    end
  end
  methods(Static)
  end
end
