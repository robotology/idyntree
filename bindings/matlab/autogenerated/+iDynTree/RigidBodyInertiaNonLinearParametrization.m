classdef RigidBodyInertiaNonLinearParametrization < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = mass(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(748, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(749, self, varargin{1});
      end
    end
    function varargout = com(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(750, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(751, self, varargin{1});
      end
    end
    function varargout = link_R_centroidal(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(752, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(753, self, varargin{1});
      end
    end
    function varargout = centralSecondMomentOfMass(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(754, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(755, self, varargin{1});
      end
    end
    function varargout = getLinkCentroidalTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(756, self, varargin{:});
    end
    function varargout = fromRigidBodyInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(757, self, varargin{:});
    end
    function varargout = fromInertialParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(758, self, varargin{:});
    end
    function varargout = toRigidBodyInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(759, self, varargin{:});
    end
    function varargout = isPhysicallyConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(760, self, varargin{:});
    end
    function varargout = asVectorWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(761, self, varargin{:});
    end
    function varargout = fromVectorWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(762, self, varargin{:});
    end
    function varargout = getGradientWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(763, self, varargin{:});
    end
    function self = RigidBodyInertiaNonLinearParametrization(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(764, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(765, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
