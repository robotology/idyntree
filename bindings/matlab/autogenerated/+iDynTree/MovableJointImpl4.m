classdef MovableJointImpl4 < iDynTree.IJoint
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1005, self);
        self.SwigClear();
      end
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1006, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1007, self, varargin{:});
    end
    function varargout = setIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1008, self, varargin{:});
    end
    function varargout = getIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1009, self, varargin{:});
    end
    function varargout = setPosCoordsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1010, self, varargin{:});
    end
    function varargout = getPosCoordsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1011, self, varargin{:});
    end
    function varargout = setDOFsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1012, self, varargin{:});
    end
    function varargout = getDOFsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1013, self, varargin{:});
    end
    function self = MovableJointImpl4(varargin)
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
