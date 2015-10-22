classdef SpatialMotionVectorBase < iDynTree.IVector
  methods
    function self = SpatialMotionVectorBase(varargin)
      self@iDynTree.IVector('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(327, varargin{:});
        tmp = iDynTreeMATLAB_wrap(327, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(328, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getLinearVec3(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(329, self, varargin{:});
    end
    function varargout = getAngularVec3(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(330, self, varargin{:});
    end
    function varargout = setLinearVec3(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(331, self, varargin{:});
    end
    function varargout = setAngularVec3(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(332, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(333, self, varargin{:});
    end
    function varargout = getVal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(334, self, varargin{:});
    end
    function varargout = setVal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(335, self, varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(336, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(337, self, varargin{:});
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(338, self, varargin{:});
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(339, self, varargin{:});
    end
    function varargout = dot(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(342, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(343, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(344, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(345, self, varargin{:});
    end
    function varargout = asVector(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(347, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(348, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(349, self, varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(350, self, varargin{:});
    end
    function varargout = fromMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(351, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(340, varargin{:});
    end
    function varargout = inverse(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(341, varargin{:});
    end
    function varargout = Zero(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(346, varargin{:});
    end
  end
end
