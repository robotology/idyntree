classdef MovableJointImpl5 < iDynTree.IJoint
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(859, self);
        self.swigPtr=[];
      end
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(860, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(861, self, varargin{:});
    end
    function varargout = setIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(862, self, varargin{:});
    end
    function varargout = getIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(863, self, varargin{:});
    end
    function varargout = setPosCoordsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(864, self, varargin{:});
    end
    function varargout = getPosCoordsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(865, self, varargin{:});
    end
    function varargout = setDOFsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(866, self, varargin{:});
    end
    function varargout = getDOFsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(867, self, varargin{:});
    end
    function self = MovableJointImpl5(varargin)
      self@iDynTree.IJoint(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
