classdef Polygon < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = m_vertices(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(2112, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(2113, self, varargin{1});
      end
    end
    function self = Polygon(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(2114, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = setNrOfVertices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2115, self, varargin{:});
    end
    function varargout = getNrOfVertices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2116, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2117, self, varargin{:});
    end
    function varargout = applyTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2118, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2119, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(2121, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = XYRectangleFromOffsets(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(2120, varargin{:});
    end
  end
end
