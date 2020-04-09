classdef UnknownWrenchContact < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = UnknownWrenchContact(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1604, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = unknownType(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1605, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1606, self, varargin{1});
      end
    end
    function varargout = contactPoint(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1607, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1608, self, varargin{1});
      end
    end
    function varargout = forceDirection(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1609, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1610, self, varargin{1});
      end
    end
    function varargout = knownWrench(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1611, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1612, self, varargin{1});
      end
    end
    function varargout = contactId(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1613, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1614, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1615, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
