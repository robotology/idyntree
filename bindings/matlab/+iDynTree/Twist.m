classdef Twist < iDynTree.SpatialMotionVector
  methods
    function self = Twist(varargin)
      self@iDynTree.SpatialMotionVector('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(425, varargin{:});
        tmp = iDynTreeMATLAB_wrap(425, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(426, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = plus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(427, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(428, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(429, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(430, self, varargin{:});
    end
  end
  methods(Static)
  end
end
