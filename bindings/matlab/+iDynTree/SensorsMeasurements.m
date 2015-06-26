classdef SensorsMeasurements < SwigRef
  methods
    function self = SensorsMeasurements(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(237,'new_SensorsMeasurements',varargin{:});
        tmp = iDynTreeMATLAB_wrap(237,'new_SensorsMeasurements',varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(238,'delete_SensorsMeasurements',self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = setNrOfSensors(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(239,'SensorsMeasurements_setNrOfSensors',self,varargin{:});
    end
    function varargout = getNrOfSensors(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(240,'SensorsMeasurements_getNrOfSensors',self,varargin{:});
    end
    function varargout = setMeasurement(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(241,'SensorsMeasurements_setMeasurement',self,varargin{:});
    end
    function varargout = getMeasurement(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(242,'SensorsMeasurements_getMeasurement',self,varargin{:});
    end
  end
  methods(Static)
  end
end
