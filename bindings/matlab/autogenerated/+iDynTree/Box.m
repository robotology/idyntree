classdef Box < iDynTree.SolidShape
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1043, self);
        self.swigPtr=[];
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1044, self, varargin{:});
    end
    function varargout = x(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1045, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1046, self, varargin{1});
      end
    end
    function varargout = y(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1047, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1048, self, varargin{1});
      end
    end
    function varargout = z(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1049, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1050, self, varargin{1});
      end
    end
    function self = Box(varargin)
      self@iDynTree.SolidShape(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1051, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
  end
  methods(Static)
  end
end
