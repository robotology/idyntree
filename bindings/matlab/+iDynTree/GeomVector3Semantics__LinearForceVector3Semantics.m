classdef GeomVector3Semantics__LinearForceVector3Semantics < SwigRef
  methods
    function self = GeomVector3Semantics__LinearForceVector3Semantics(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(252, varargin{:});
        tmp = iDynTreeMATLAB_wrap(252, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(253, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getBody(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(254, self, varargin{:});
    end
    function varargout = getRefBody(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(255, self, varargin{:});
    end
    function varargout = getCoordinateFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(256, self, varargin{:});
    end
    function varargout = isUnknown(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(257, self, varargin{:});
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(258, self, varargin{:});
    end
    function varargout = dot(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(261, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(259, varargin{:});
    end
    function varargout = inverse(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(260, varargin{:});
    end
  end
end
