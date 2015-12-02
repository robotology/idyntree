classdef LinearMotionVector3Semantics < iDynTree.GeomVector3Semantics__LinearMotionVector3Semantics
  methods
    function self = LinearMotionVector3Semantics(varargin)
      self@iDynTree.GeomVector3Semantics__LinearMotionVector3Semantics('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(285, varargin{:});
        tmp = iDynTreeMATLAB_wrap(285, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(286, self, varargin{:});
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(288, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(287, varargin{:});
    end
  end
end
