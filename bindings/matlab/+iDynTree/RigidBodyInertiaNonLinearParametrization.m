classdef RigidBodyInertiaNonLinearParametrization < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = mass(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(637, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(638, self, varargin{1});
      end
    end
    function varargout = com(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(639, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(640, self, varargin{1});
      end
    end
    function varargout = link_R_centroidal(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(641, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(642, self, varargin{1});
      end
    end
    function varargout = centralSecondMomentOfMass(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(643, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(644, self, varargin{1});
      end
    end
    function varargout = getLinkCentroidalTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(645, self, varargin{:});
    end
    function varargout = fromRigidBodyInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(646, self, varargin{:});
    end
    function varargout = fromInertialParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(647, self, varargin{:});
    end
    function varargout = toRigidBodyInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(648, self, varargin{:});
    end
    function varargout = isPhysicallyConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(649, self, varargin{:});
    end
    function varargout = asVectorWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(650, self, varargin{:});
    end
    function varargout = fromVectorWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(651, self, varargin{:});
    end
    function varargout = getGradientWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(652, self, varargin{:});
    end
    function self = RigidBodyInertiaNonLinearParametrization(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(653, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(654, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
