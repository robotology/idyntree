classdef SimpleLeggedOdometry < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = SimpleLeggedOdometry(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1527, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1528, self);
        self.SwigClear();
      end
    end
    function varargout = setModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1529, self, varargin{:});
    end
    function varargout = loadModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1530, self, varargin{:});
    end
    function varargout = loadModelFromFileWithSpecifiedDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1531, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1532, self, varargin{:});
    end
    function varargout = updateKinematics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1533, self, varargin{:});
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1534, self, varargin{:});
    end
    function varargout = changeFixedFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1535, self, varargin{:});
    end
    function varargout = getCurrentFixedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1536, self, varargin{:});
    end
    function varargout = getWorldLinkTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1537, self, varargin{:});
    end
  end
  methods(Static)
  end
end
