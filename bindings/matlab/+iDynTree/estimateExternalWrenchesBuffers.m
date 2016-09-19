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
        tmp = iDynTreeMEX(1253, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1254, self, varargin{:});
    end
    function varargout = getNrOfSubModels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1255, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1256, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1257, self, varargin{:});
    end
    function varargout = A(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1258, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1259, self, varargin{1});
      end
    end
    function varargout = x(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1260, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1261, self, varargin{1});
      end
    end
    function varargout = b(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1262, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1263, self, varargin{1});
      end
    end
    function varargout = pinvA(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1264, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1265, self, varargin{1});
      end
    end
    function varargout = b_contacts_subtree(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1266, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1267, self, varargin{1});
      end
    end
    function varargout = subModelBase_H_link(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1268, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1269, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1270, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
