classdef UnknownWrenchContact < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = UnknownWrenchContact(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1473, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = unknownType(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1474, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1475, self, varargin{1});
      end
    end
    function varargout = contactPoint(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1476, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1477, self, varargin{1});
      end
    end
    function varargout = forceDirection(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1478, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1479, self, varargin{1});
      end
    end
    function varargout = knownWrench(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1480, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1481, self, varargin{1});
      end
    end
    function varargout = contactId(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1482, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1483, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1484, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
