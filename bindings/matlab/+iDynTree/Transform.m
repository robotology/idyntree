classdef Transform < SwigRef
  methods
    function self = Transform(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(334, varargin{:});
        tmp = iDynTreeMATLAB_wrap(334, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(335, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getSemantics(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(336, self, varargin{:});
    end
    function varargout = getRotation(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(337, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(338, self, varargin{:});
    end
    function varargout = setRotation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(339, self, varargin{:});
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(340, self, varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(343, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(344, self, varargin{:});
    end
    function varargout = asHomogeneousTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(346, self, varargin{:});
    end
    function varargout = asAdjointTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(347, self, varargin{:});
    end
    function varargout = asAdjointTransformWrench(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(348, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(349, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(350, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(341, varargin{:});
    end
    function varargout = inverse2(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(342, varargin{:});
    end
    function varargout = Identity(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(345, varargin{:});
    end
  end
end
