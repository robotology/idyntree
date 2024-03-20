classdef estimateExternalWrenchesBuffers < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = estimateExternalWrenchesBuffers(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1594, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1595, self, varargin{:});
    end
    function varargout = getNrOfSubModels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1596, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1597, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1598, self, varargin{:});
    end
    function varargout = A(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1599, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1600, self, varargin{1});
      end
    end
    function varargout = x(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1601, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1602, self, varargin{1});
      end
    end
    function varargout = b(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1603, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1604, self, varargin{1});
      end
    end
    function varargout = b_contacts_subtree(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1605, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1606, self, varargin{1});
      end
    end
    function varargout = subModelBase_H_link(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1607, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1608, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1609, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
