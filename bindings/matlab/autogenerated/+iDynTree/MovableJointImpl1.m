classdef MovableJointImpl1 < iDynTree.IJoint
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1027, self);
        self.SwigClear();
      end
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1028, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1029, self, varargin{:});
    end
    function varargout = setIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1030, self, varargin{:});
    end
    function varargout = getIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1031, self, varargin{:});
    end
    function varargout = setPosCoordsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1032, self, varargin{:});
    end
    function varargout = getPosCoordsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1033, self, varargin{:});
    end
    function varargout = setDOFsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1034, self, varargin{:});
    end
    function varargout = getDOFsOffset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1035, self, varargin{:});
    end
    function self = MovableJointImpl1(varargin)
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
