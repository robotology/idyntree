classdef Box < iDynTree.SolidShape
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(956, self);
        self.swigPtr=[];
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(957, self, varargin{:});
    end
    function varargout = x(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(958, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(959, self, varargin{1});
      end
    end
    function varargout = y(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(960, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(961, self, varargin{1});
      end
    end
    function varargout = z(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(962, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(963, self, varargin{1});
      end
    end
    function self = Box(varargin)
      self@iDynTree.SolidShape(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(964, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
  end
  methods(Static)
  end
end
