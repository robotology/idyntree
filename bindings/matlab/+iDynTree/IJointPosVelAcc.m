classdef IJointPosVelAcc < iDynTree.IJointPosVel
  methods
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(358, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = acc(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(359, self, varargin{:});
    end
    function self = IJointPosVelAcc(varargin)
      self@iDynTree.IJointPosVel('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
