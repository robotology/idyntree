classdef ExternalMesh < iDynTree.SolidShape
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1059, self);
        self.swigPtr=[];
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1060, self, varargin{:});
    end
    function varargout = filename(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1061, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1062, self, varargin{1});
      end
    end
    function varargout = scale(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1063, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1064, self, varargin{1});
      end
    end
    function self = ExternalMesh(varargin)
      self@iDynTree.SolidShape(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1065, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
  end
  methods(Static)
  end
end
