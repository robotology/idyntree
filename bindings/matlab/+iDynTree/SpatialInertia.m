classdef SpatialInertia < iDynTree.SpatialInertiaRaw
  methods
    function self = SpatialInertia(varargin)
      self@iDynTree.SpatialInertiaRaw('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(480, varargin{:});
        tmp = iDynTreeMATLAB_wrap(480, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(481, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = asMatrix(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(483, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(484, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(485, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = combine(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(482, varargin{:});
    end
  end
end
