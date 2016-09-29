classdef ArticulatedBodyAlgorithmInternalBuffers < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = ArticulatedBodyAlgorithmInternalBuffers(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1103, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1104, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1105, self, varargin{:});
    end
    function varargout = S(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1106, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1107, self, varargin{1});
      end
    end
    function varargout = U(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1108, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1109, self, varargin{1});
      end
    end
    function varargout = D(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1110, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1111, self, varargin{1});
      end
    end
    function varargout = u(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1112, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1113, self, varargin{1});
      end
    end
    function varargout = linksVel(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1114, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1115, self, varargin{1});
      end
    end
    function varargout = linksBiasAcceleration(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1116, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1117, self, varargin{1});
      end
    end
    function varargout = linksAccelerations(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1118, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1119, self, varargin{1});
      end
    end
    function varargout = linkABIs(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1120, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1121, self, varargin{1});
      end
    end
    function varargout = linksBiasWrench(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1122, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1123, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1124, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
