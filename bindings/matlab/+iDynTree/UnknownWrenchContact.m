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
        tmp = iDynTreeMEX(1208, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = unknownType(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1209, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1210, self, varargin{1});
      end
    end
    function varargout = contactPoint(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1211, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1212, self, varargin{1});
      end
    end
    function varargout = forceDirection(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1213, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1214, self, varargin{1});
      end
    end
    function varargout = contactId(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1215, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1216, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1217, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
