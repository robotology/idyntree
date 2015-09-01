classdef IJointPosVel < iDynTree.IJointPos
  methods
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(586, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = vel(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(587, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(588, self, varargin{:});
    end
    function self = IJointPosVel(varargin)
      self@iDynTree.IJointPos('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
