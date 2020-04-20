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
        tmp = iDynTreeMEX(1207, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = copy(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1208, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1209, self);
        self.SwigClear();
      end
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1210, self, varargin{:});
    end
    function varargout = getLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1211, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1212, self, varargin{:});
    end
    function varargout = isValidLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1213, self, varargin{:});
    end
    function varargout = getLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1214, self, varargin{:});
    end
    function varargout = addLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1215, self, varargin{:});
    end
    function varargout = getNrOfJoints(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1216, self, varargin{:});
    end
    function varargout = getJointName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1217, self, varargin{:});
    end
    function varargout = getTotalMass(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1218, self, varargin{:});
    end
    function varargout = getJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1219, self, varargin{:});
    end
    function varargout = getJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1220, self, varargin{:});
    end
    function varargout = isValidJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1221, self, varargin{:});
    end
    function varargout = isLinkNameUsed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1222, self, varargin{:});
    end
    function varargout = isJointNameUsed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1223, self, varargin{:});
    end
    function varargout = isFrameNameUsed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1224, self, varargin{:});
    end
    function varargout = addJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1225, self, varargin{:});
    end
    function varargout = addJointAndLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1226, self, varargin{:});
    end
    function varargout = insertLinkToExistingJointAndAddJointForDisplacedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1227, self, varargin{:});
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1228, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1229, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1230, self, varargin{:});
    end
    function varargout = addAdditionalFrameToLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1231, self, varargin{:});
    end
    function varargout = getFrameName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1232, self, varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1233, self, varargin{:});
    end
    function varargout = isValidFrameIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1234, self, varargin{:});
    end
    function varargout = getFrameTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1235, self, varargin{:});
    end
    function varargout = getFrameLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1236, self, varargin{:});
    end
    function varargout = getLinkAdditionalFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1237, self, varargin{:});
    end
    function varargout = getNrOfNeighbors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1238, self, varargin{:});
    end
    function varargout = getNeighbor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1239, self, varargin{:});
    end
    function varargout = setDefaultBaseLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1240, self, varargin{:});
    end
    function varargout = getDefaultBaseLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1241, self, varargin{:});
    end
    function varargout = computeFullTreeTraversal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1242, self, varargin{:});
    end
    function varargout = getInertialParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1243, self, varargin{:});
    end
    function varargout = updateInertialParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1244, self, varargin{:});
    end
    function varargout = visualSolidShapes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1245, self, varargin{:});
    end
    function varargout = collisionSolidShapes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1246, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1247, self, varargin{:});
    end
  end
  methods(Static)
  end
end
