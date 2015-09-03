classdef ForceVector3Semantics__LinearForceVector3Semantics < iDynTree.GeomVector3Semantics__LinearForceVector3Semantics
  methods
    function self = ForceVector3Semantics__LinearForceVector3Semantics(varargin)
      self@iDynTree.GeomVector3Semantics__LinearForceVector3Semantics('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(276, varargin{:});
        tmp = iDynTreeMATLAB_wrap(276, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(277, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(278, varargin{:});
    end
    function varargout = inverse(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(279, varargin{:});
    end
  end
end
