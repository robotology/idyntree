classdef BerdyOptions < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = BerdyOptions(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1423, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = berdyVariant(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1424, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1425, self, varargin{1});
      end
    end
    function varargout = includeAllNetExternalWrenchesAsDynamicVariables(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1426, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1427, self, varargin{1});
      end
    end
    function varargout = includeAllJointAccelerationsAsSensors(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1428, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1429, self, varargin{1});
      end
    end
    function varargout = includeAllJointTorquesAsSensors(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1430, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1431, self, varargin{1});
      end
    end
    function varargout = includeAllNetExternalWrenchesAsSensors(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1432, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1433, self, varargin{1});
      end
    end
    function varargout = includeFixedBaseExternalWrench(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1434, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1435, self, varargin{1});
      end
    end
    function varargout = jointOnWhichTheInternalWrenchIsMeasured(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1436, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1437, self, varargin{1});
      end
    end
    function varargout = baseLink(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1438, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1439, self, varargin{1});
      end
    end
    function varargout = checkConsistency(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1440, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1441, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
