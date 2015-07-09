classdef Sensor < SwigRef
  methods
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(314,'delete_Sensor',self);
        self.swigOwn=false;
      end
    end
    function varargout = getName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(315,'Sensor_getName',self,varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(316,'Sensor_getSensorType',self,varargin{:});
    end
    function varargout = getParent(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(317,'Sensor_getParent',self,varargin{:});
    end
    function varargout = getParentIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(318,'Sensor_getParentIndex',self,varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(319,'Sensor_isValid',self,varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(320,'Sensor_clone',self,varargin{:});
    end
    function self = Sensor(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        error('No matching constructor');
      end
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
