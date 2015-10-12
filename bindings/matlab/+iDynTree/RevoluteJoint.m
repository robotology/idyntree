classdef RevoluteJoint < iDynTree.MovableJointImpl1
  methods
    function self = RevoluteJoint(varargin)
      self@iDynTree.MovableJointImpl1('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(618, varargin{:});
        tmp = iDynTreeMATLAB_wrap(618, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(619, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(620, self, varargin{:});
    end
    function varargout = setAttachedLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(621, self, varargin{:});
    end
    function varargout = setRestTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(622, self, varargin{:});
    end
    function varargout = setAxis(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(623, self, varargin{:});
    end
    function varargout = getFirstAttachedLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(624, self, varargin{:});
    end
    function varargout = getSecondAttachedLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(625, self, varargin{:});
    end
    function varargout = getAxis(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(626, self, varargin{:});
    end
    function varargout = getTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(627, self, varargin{:});
    end
    function varargout = computeLinkPosVelAcc(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(628, self, varargin{:});
    end
    function varargout = computeLinkVelAcc(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(629, self, varargin{:});
    end
    function varargout = computeJointTorque(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(630, self, varargin{:});
    end
  end
  methods(Static)
  end
end
