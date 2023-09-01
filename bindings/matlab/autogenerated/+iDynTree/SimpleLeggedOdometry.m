classdef SimpleLeggedOdometry < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = SimpleLeggedOdometry(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1578, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1579, self);
        self.SwigClear();
      end
    end
    function varargout = setModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1580, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1581, self, varargin{:});
    end
    function varargout = updateKinematics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1582, self, varargin{:});
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1583, self, varargin{:});
    end
    function varargout = changeFixedFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1584, self, varargin{:});
    end
    function varargout = getCurrentFixedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1585, self, varargin{:});
    end
    function varargout = getWorldLinkTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1586, self, varargin{:});
    end
    function varargout = getWorldFrameTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1587, self, varargin{:});
    end
  end
  methods(Static)
  end
end
