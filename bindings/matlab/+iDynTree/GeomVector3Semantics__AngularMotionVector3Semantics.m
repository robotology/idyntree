classdef GeomVector3Semantics__AngularMotionVector3Semantics < SwigRef
  methods
    function self = GeomVector3Semantics__AngularMotionVector3Semantics(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(242, varargin{:});
        tmp = iDynTreeMATLAB_wrap(242, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(243, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getBody(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(244, self, varargin{:});
    end
    function varargout = getRefBody(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(245, self, varargin{:});
    end
    function varargout = getCoordinateFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(246, self, varargin{:});
    end
    function varargout = isUnknown(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(247, self, varargin{:});
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(248, self, varargin{:});
    end
    function varargout = dot(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(251, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(249, varargin{:});
    end
    function varargout = inverse(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(250, varargin{:});
    end
  end
end
