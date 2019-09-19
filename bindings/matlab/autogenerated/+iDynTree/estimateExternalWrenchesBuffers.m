classdef estimateExternalWrenchesBuffers < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = estimateExternalWrenchesBuffers(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1513, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1514, self, varargin{:});
    end
    function varargout = getNrOfSubModels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1515, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1516, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1517, self, varargin{:});
    end
    function varargout = A(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1518, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1519, self, varargin{1});
      end
    end
    function varargout = x(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1520, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1521, self, varargin{1});
      end
    end
    function varargout = b(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1522, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1523, self, varargin{1});
      end
    end
    function varargout = pinvA(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1524, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1525, self, varargin{1});
      end
    end
    function varargout = b_contacts_subtree(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1526, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1527, self, varargin{1});
      end
    end
    function varargout = subModelBase_H_link(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1528, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1529, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1530, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
