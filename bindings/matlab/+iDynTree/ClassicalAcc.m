classdef ClassicalAcc < iDynTree.Vector6
  methods
    function self = ClassicalAcc(varargin)
      self@iDynTree.Vector6('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(408, varargin{:});
        tmp = iDynTreeMATLAB_wrap(408, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(409, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(410, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = Zero(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(411, varargin{:});
    end
  end
end
