classdef IJointTorque < SwigRef
  methods
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(585, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = torque(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(586, self, varargin{:});
    end
    function self = IJointTorque(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
