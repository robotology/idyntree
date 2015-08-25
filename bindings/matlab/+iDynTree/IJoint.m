classdef IJoint < SwigRef
  methods
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(387, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(388, self, varargin{:});
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(389, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(390, self, varargin{:});
    end
    function varargout = setAttachedLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(391, self, varargin{:});
    end
    function varargout = setRestTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(392, self, varargin{:});
    end
    function varargout = getFirstAttachedLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(393, self, varargin{:});
    end
    function varargout = getSecondAttachedLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(394, self, varargin{:});
    end
    function varargout = getTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(395, self, varargin{:});
    end
    function varargout = computeLinkPosVelAcc(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(396, self, varargin{:});
    end
    function varargout = computeLinkVelAcc(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(397, self, varargin{:});
    end
    function varargout = computeJointTorque(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(398, self, varargin{:});
    end
    function self = IJoint(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
