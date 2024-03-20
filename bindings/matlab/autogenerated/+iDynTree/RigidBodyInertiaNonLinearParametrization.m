classdef RigidBodyInertiaNonLinearParametrization < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = mass(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(663, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(664, self, varargin{1});
      end
    end
    function varargout = com(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(665, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(666, self, varargin{1});
      end
    end
    function varargout = link_R_centroidal(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(667, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(668, self, varargin{1});
      end
    end
    function varargout = centralSecondMomentOfMass(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(669, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(670, self, varargin{1});
      end
    end
    function varargout = getLinkCentroidalTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(671, self, varargin{:});
    end
    function varargout = fromRigidBodyInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(672, self, varargin{:});
    end
    function varargout = fromInertialParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(673, self, varargin{:});
    end
    function varargout = toRigidBodyInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(674, self, varargin{:});
    end
    function varargout = isPhysicallyConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(675, self, varargin{:});
    end
    function varargout = asVectorWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(676, self, varargin{:});
    end
    function varargout = fromVectorWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(677, self, varargin{:});
    end
    function varargout = getGradientWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(678, self, varargin{:});
    end
    function self = RigidBodyInertiaNonLinearParametrization(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(679, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(680, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
