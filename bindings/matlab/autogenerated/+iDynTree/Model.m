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
        tmp = iDynTreeMEX(1152, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = copy(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1153, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1154, self);
        self.SwigClear();
      end
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1155, self, varargin{:});
    end
    function varargout = getLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1156, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1157, self, varargin{:});
    end
    function varargout = isValidLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1158, self, varargin{:});
    end
    function varargout = getLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1159, self, varargin{:});
    end
    function varargout = addLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1160, self, varargin{:});
    end
    function varargout = getNrOfJoints(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1161, self, varargin{:});
    end
    function varargout = getJointName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1162, self, varargin{:});
    end
    function varargout = getTotalMass(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1163, self, varargin{:});
    end
    function varargout = getJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1164, self, varargin{:});
    end
    function varargout = getJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1165, self, varargin{:});
    end
    function varargout = isValidJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1166, self, varargin{:});
    end
    function varargout = isLinkNameUsed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1167, self, varargin{:});
    end
    function varargout = isJointNameUsed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1168, self, varargin{:});
    end
    function varargout = isFrameNameUsed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1169, self, varargin{:});
    end
    function varargout = addJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1170, self, varargin{:});
    end
    function varargout = addJointAndLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1171, self, varargin{:});
    end
    function varargout = insertLinkToExistingJointAndAddJointForDisplacedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1172, self, varargin{:});
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1173, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1174, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1175, self, varargin{:});
    end
    function varargout = addAdditionalFrameToLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1176, self, varargin{:});
    end
    function varargout = getFrameName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1177, self, varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1178, self, varargin{:});
    end
    function varargout = isValidFrameIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1179, self, varargin{:});
    end
    function varargout = getFrameTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1180, self, varargin{:});
    end
    function varargout = getFrameLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1181, self, varargin{:});
    end
    function varargout = getLinkAdditionalFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1182, self, varargin{:});
    end
    function varargout = getNrOfNeighbors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1183, self, varargin{:});
    end
    function varargout = getNeighbor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1184, self, varargin{:});
    end
    function varargout = setDefaultBaseLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1185, self, varargin{:});
    end
    function varargout = getDefaultBaseLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1186, self, varargin{:});
    end
    function varargout = computeFullTreeTraversal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1187, self, varargin{:});
    end
    function varargout = getInertialParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1188, self, varargin{:});
    end
    function varargout = updateInertialParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1189, self, varargin{:});
    end
    function varargout = visualSolidShapes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1190, self, varargin{:});
    end
    function varargout = collisionSolidShapes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1191, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1192, self, varargin{:});
    end
  end
  methods(Static)
  end
end
