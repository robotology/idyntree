classdef IRawMatrix < iDynTree.IMatrix
  methods
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(9, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = data(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(10, self, varargin{:});
    end
    function self = IRawMatrix(varargin)
      self@iDynTree.IMatrix('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
