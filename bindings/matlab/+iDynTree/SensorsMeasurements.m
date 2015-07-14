classdef SensorsMeasurements < SwigRef
  methods
    function self = SensorsMeasurements(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(293,'new_SensorsMeasurements',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(293,'new_SensorsMeasurements',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(294,'delete_SensorsMeasurements',self);
        self.swigOwn=false;
      end
    end
    function varargout = setNrOfSensors(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(295,'SensorsMeasurements_setNrOfSensors',self,varargin{:});
    end
    function varargout = getNrOfSensors(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(296,'SensorsMeasurements_getNrOfSensors',self,varargin{:});
    end
    function varargout = setMeasurement(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(297,'SensorsMeasurements_setMeasurement',self,varargin{:});
    end
    function varargout = getMeasurement(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(298,'SensorsMeasurements_getMeasurement',self,varargin{:});
    end
    function [v,ok] = swig_fieldsref(self,i)
      v = [];
      ok = false;
      switch i
      end
    end
    function [self,ok] = swig_fieldasgn(self,i,v)
      switch i
      end
    end
  end
  methods(Static)
  end
end
