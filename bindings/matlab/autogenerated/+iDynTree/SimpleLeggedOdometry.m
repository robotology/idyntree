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
        tmp = iDynTreeMEX(1392, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1393, self);
        self.swigPtr=[];
      end
    end
    function varargout = setModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1394, self, varargin{:});
    end
    function varargout = loadModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1395, self, varargin{:});
    end
    function varargout = loadModelFromFileWithSpecifiedDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1396, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1397, self, varargin{:});
    end
    function varargout = updateKinematics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1398, self, varargin{:});
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1399, self, varargin{:});
    end
    function varargout = changeFixedFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1400, self, varargin{:});
    end
    function varargout = getCurrentFixedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1401, self, varargin{:});
    end
    function varargout = getWorldLinkTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1402, self, varargin{:});
    end
  end
  methods(Static)
  end
end
