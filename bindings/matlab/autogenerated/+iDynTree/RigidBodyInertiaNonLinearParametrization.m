classdef RigidBodyInertiaNonLinearParametrization < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = mass(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(600, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(601, self, varargin{1});
      end
    end
    function varargout = com(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(602, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(603, self, varargin{1});
      end
    end
    function varargout = link_R_centroidal(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(604, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(605, self, varargin{1});
      end
    end
    function varargout = centralSecondMomentOfMass(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(606, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(607, self, varargin{1});
      end
    end
    function varargout = getLinkCentroidalTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(608, self, varargin{:});
    end
    function varargout = fromRigidBodyInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(609, self, varargin{:});
    end
    function varargout = fromInertialParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(610, self, varargin{:});
    end
    function varargout = toRigidBodyInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(611, self, varargin{:});
    end
    function varargout = isPhysicallyConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(612, self, varargin{:});
    end
    function varargout = asVectorWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(613, self, varargin{:});
    end
    function varargout = fromVectorWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(614, self, varargin{:});
    end
    function varargout = getGradientWithRotationAsVec(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(615, self, varargin{:});
    end
    function self = RigidBodyInertiaNonLinearParametrization(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(616, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(617, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
