classdef MovableJointImpl3 < iDynTree.IJoint
  methods
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(640, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(641, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(642, self, varargin{:});
    end
    function varargout = setIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(643, self, varargin{:});
    end
    function varargout = getIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(644, self, varargin{:});
    end
    function varargout = setPosCoordsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(645, self, varargin{:});
    end
    function varargout = getPosCoordsOffset(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(646, self, varargin{:});
    end
    function varargout = setDOFsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(647, self, varargin{:});
    end
    function varargout = getDOFsOffset(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(648, self, varargin{:});
    end
    function self = MovableJointImpl3(varargin)
      self@iDynTree.IJoint('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
