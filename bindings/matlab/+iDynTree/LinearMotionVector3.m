classdef LinearMotionVector3 < iDynTree.MotionVector3__LinearMotionVector3
  methods
    function self = LinearMotionVector3(varargin)
      self@iDynTree.MotionVector3__LinearMotionVector3('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(298, varargin{:});
        tmp = iDynTreeMATLAB_wrap(298, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(299, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(300, self, varargin{:});
    end
  end
  methods(Static)
  end
end
