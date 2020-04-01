classdef RigidBodyInertiaNonLinearParametrization < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = mass(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(723, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(724, self, varargin{1});
      end
    end
    function varargout = com(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(725, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(726, self, varargin{1});
      end
    end
    function varargout = link_R_centroidal(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(727, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(728, self, varargin{1});
      end
    end
    function varargout = centralSecondMomentOfMass(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(729, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(730, self, varargin{1});
      end
    end
    function varargout = getLinkCentroidalTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(731, self, varargin{:});
    end
    function varargout = fromRigidBodyInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(732, self, varargin{:});
    end
    function varargout = fromInertialParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(733, self, varargin{:});
    end
    function varargout = toRigidBodyInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(734, self, varargin{:});
    end
    function varargout = isPhysicallyConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(735, self, varargin{:});
    end
    function varargout = asVectorWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(736, self, varargin{:});
    end
    function varargout = fromVectorWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(737, self, varargin{:});
    end
    function varargout = getGradientWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(738, self, varargin{:});
    end
    function self = RigidBodyInertiaNonLinearParametrization(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(739, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(740, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
