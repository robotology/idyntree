classdef SensorsList < SwigRef
  methods
    function self = SensorsList(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(743, varargin{:});
        tmp = iDynTreeMATLAB_wrap(743, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(744, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = addSensor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(745, self, varargin{:});
    end
    function varargout = getNrOfSensors(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(746, self, varargin{:});
    end
    function varargout = getSensorIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(747, self, varargin{:});
    end
    function varargout = getSensor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(748, self, varargin{:});
    end
    function varargout = getSixAxisForceTorqueSensor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(749, self, varargin{:});
    end
  end
  methods(Static)
  end
end
