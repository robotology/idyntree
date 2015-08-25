classdef Model < SwigRef
  methods
    function self = Model(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(457, varargin{:});
        tmp = iDynTreeMATLAB_wrap(457, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(458, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(459, self, varargin{:});
    end
    function varargout = getLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(460, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(461, self, varargin{:});
    end
    function varargout = addLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(462, self, varargin{:});
    end
    function varargout = getNrOfJoints(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(463, self, varargin{:});
    end
    function varargout = getJointName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(464, self, varargin{:});
    end
    function varargout = getJointIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(465, self, varargin{:});
    end
    function varargout = addJoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(466, self, varargin{:});
    end
    function varargout = getNrOfNeighbors(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(467, self, varargin{:});
    end
    function varargout = getNeighbor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(468, self, varargin{:});
    end
    function varargout = setDefaultBaseLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(469, self, varargin{:});
    end
    function varargout = getDefaultBaseLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(470, self, varargin{:});
    end
    function varargout = computeFullTreeTraversal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(471, self, varargin{:});
    end
  end
  methods(Static)
  end
end
