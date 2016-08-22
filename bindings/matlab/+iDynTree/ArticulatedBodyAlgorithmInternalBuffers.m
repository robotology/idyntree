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
        tmp = iDynTreeMEX(1033, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1034, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1035, self, varargin{:});
    end
    function varargout = S(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1036, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1037, self, varargin{1});
      end
    end
    function varargout = U(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1038, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1039, self, varargin{1});
      end
    end
    function varargout = D(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1040, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1041, self, varargin{1});
      end
    end
    function varargout = u(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1042, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1043, self, varargin{1});
      end
    end
    function varargout = linksVel(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1044, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1045, self, varargin{1});
      end
    end
    function varargout = linksBiasAcceleration(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1046, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1047, self, varargin{1});
      end
    end
    function varargout = linksAccelerations(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1048, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1049, self, varargin{1});
      end
    end
    function varargout = linkABIs(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1050, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1051, self, varargin{1});
      end
    end
    function varargout = linksBiasWrench(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1052, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1053, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1054, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
