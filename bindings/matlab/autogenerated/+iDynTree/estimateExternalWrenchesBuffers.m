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
        tmp = iDynTreeMEX(1434, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1435, self, varargin{:});
    end
    function varargout = getNrOfSubModels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1436, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1437, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1438, self, varargin{:});
    end
    function varargout = A(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1439, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1440, self, varargin{1});
      end
    end
    function varargout = x(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1441, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1442, self, varargin{1});
      end
    end
    function varargout = b(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1443, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1444, self, varargin{1});
      end
    end
    function varargout = pinvA(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1445, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1446, self, varargin{1});
      end
    end
    function varargout = b_contacts_subtree(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1447, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1448, self, varargin{1});
      end
    end
    function varargout = subModelBase_H_link(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1449, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1450, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1451, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
