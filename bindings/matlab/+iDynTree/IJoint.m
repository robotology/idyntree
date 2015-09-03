classdef IJoint < SwigRef
  methods
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(555, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(556, self, varargin{:});
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(557, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(558, self, varargin{:});
    end
    function varargout = setAttachedLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(559, self, varargin{:});
    end
    function varargout = setRestTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(560, self, varargin{:});
    end
    function varargout = getFirstAttachedLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(561, self, varargin{:});
    end
    function varargout = getSecondAttachedLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(562, self, varargin{:});
    end
    function varargout = getTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(563, self, varargin{:});
    end
    function varargout = computeLinkPosVelAcc(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(564, self, varargin{:});
    end
    function varargout = computeLinkVelAcc(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(565, self, varargin{:});
    end
    function varargout = computeJointTorque(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(566, self, varargin{:});
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
