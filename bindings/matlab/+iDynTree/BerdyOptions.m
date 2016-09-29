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
        tmp = iDynTreeMEX(1331, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = berdyVariant(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1332, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1333, self, varargin{1});
      end
    end
    function varargout = includeAllNetExternalWrenchesAsDynamicVariables(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1334, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1335, self, varargin{1});
      end
    end
    function varargout = includeAllJointAccelerationsAsSensors(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1336, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1337, self, varargin{1});
      end
    end
    function varargout = includeAllJointTorquesAsSensors(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1338, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1339, self, varargin{1});
      end
    end
    function varargout = includeAllNetExternalWrenchesAsSensors(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1340, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1341, self, varargin{1});
      end
    end
    function varargout = includeFixedBaseExternalWrench(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1342, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1343, self, varargin{1});
      end
    end
    function varargout = jointOnWhichTheInternalWrenchIsMeasured(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1344, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1345, self, varargin{1});
      end
    end
    function varargout = baseLink(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1346, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1347, self, varargin{1});
      end
    end
    function varargout = checkConsistency(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1348, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1349, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
