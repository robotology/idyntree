classdef FreeFloatingGeneralizedTorques < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = FreeFloatingGeneralizedTorques(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(801, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(802, self, varargin{:});
    end
    function varargout = baseWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(803, self, varargin{:});
    end
    function varargout = jointTorques(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(804, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(805, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(806, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
