classdef IRawVector < iDynTree.IVector
  methods
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(16, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = data(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(17, self, varargin{:});
    end
    function self = IRawVector(varargin)
      self@iDynTree.IVector('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
