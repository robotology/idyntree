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
        tmp = iDynTreeMEX(1091, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = copy(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1092, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1093, self);
        self.swigPtr=[];
      end
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1094, self, varargin{:});
    end
    function varargout = getLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1095, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1096, self, varargin{:});
    end
    function varargout = isValidLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1097, self, varargin{:});
    end
    function varargout = getLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1098, self, varargin{:});
    end
    function varargout = addLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1099, self, varargin{:});
    end
    function varargout = getNrOfJoints(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1100, self, varargin{:});
    end
    function varargout = getJointName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1101, self, varargin{:});
    end
    function varargout = getJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1102, self, varargin{:});
    end
    function varargout = getJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1103, self, varargin{:});
    end
    function varargout = isValidJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1104, self, varargin{:});
    end
    function varargout = isLinkNameUsed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1105, self, varargin{:});
    end
    function varargout = isJointNameUsed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1106, self, varargin{:});
    end
    function varargout = isFrameNameUsed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1107, self, varargin{:});
    end
    function varargout = addJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1108, self, varargin{:});
    end
    function varargout = addJointAndLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1109, self, varargin{:});
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1110, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1111, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1112, self, varargin{:});
    end
    function varargout = addAdditionalFrameToLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1113, self, varargin{:});
    end
    function varargout = getFrameName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1114, self, varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1115, self, varargin{:});
    end
    function varargout = isValidFrameIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1116, self, varargin{:});
    end
    function varargout = getFrameTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1117, self, varargin{:});
    end
    function varargout = getFrameLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1118, self, varargin{:});
    end
    function varargout = getNrOfNeighbors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1119, self, varargin{:});
    end
    function varargout = getNeighbor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1120, self, varargin{:});
    end
    function varargout = setDefaultBaseLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1121, self, varargin{:});
    end
    function varargout = getDefaultBaseLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1122, self, varargin{:});
    end
    function varargout = computeFullTreeTraversal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1123, self, varargin{:});
    end
    function varargout = getInertialParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1124, self, varargin{:});
    end
    function varargout = updateInertialParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1125, self, varargin{:});
    end
    function varargout = visualSolidShapes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1126, self, varargin{:});
    end
    function varargout = collisionSolidShapes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1127, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1128, self, varargin{:});
    end
  end
  methods(Static)
  end
end
