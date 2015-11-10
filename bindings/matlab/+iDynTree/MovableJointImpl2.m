classdef MovableJointImpl2 < iDynTree.IJoint
  methods
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(612, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(613, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(614, self, varargin{:});
    end
    function varargout = setIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(615, self, varargin{:});
    end
    function varargout = getIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(616, self, varargin{:});
    end
    function varargout = setPosCoordsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(617, self, varargin{:});
    end
    function varargout = getPosCoordsOffset(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(618, self, varargin{:});
    end
    function varargout = setDOFsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(619, self, varargin{:});
    end
    function varargout = getDOFsOffset(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(620, self, varargin{:});
    end
    function self = MovableJointImpl2(varargin)
      self@iDynTree.IJoint('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
