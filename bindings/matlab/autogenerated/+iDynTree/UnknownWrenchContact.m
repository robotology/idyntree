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
        tmp = iDynTreeMEX(1579, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = unknownType(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1580, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1581, self, varargin{1});
      end
    end
    function varargout = contactPoint(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1582, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1583, self, varargin{1});
      end
    end
    function varargout = forceDirection(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1584, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1585, self, varargin{1});
      end
    end
    function varargout = knownWrench(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1586, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1587, self, varargin{1});
      end
    end
    function varargout = contactId(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1588, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1589, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1590, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
