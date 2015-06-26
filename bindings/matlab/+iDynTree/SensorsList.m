classdef SensorsList < SwigRef
  methods
    function self = SensorsList(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(231,'new_SensorsList',varargin{:});
        tmp = iDynTreeMATLAB_wrap(231,'new_SensorsList',varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(232,'delete_SensorsList',self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = addSensor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(233,'SensorsList_addSensor',self,varargin{:});
    end
    function varargout = getNrOfSensors(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(234,'SensorsList_getNrOfSensors',self,varargin{:});
    end
    function varargout = getSensorIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(235,'SensorsList_getSensorIndex',self,varargin{:});
    end
    function varargout = getSensor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(236,'SensorsList_getSensor',self,varargin{:});
    end
  end
  methods(Static)
  end
end
