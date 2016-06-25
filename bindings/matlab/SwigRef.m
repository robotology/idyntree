classdef SwigRef < handle
  properties(Hidden = true, Access = public) 
    swigPtr
  end
  methods(Static = true, Access = protected)
    function obj = Null()
      persistent obj_null
      if isempty(obj_null)
        obj_null = SwigRef();
      end
      obj = obj_null;
    end
  end
  methods
    function b = isnull(self)
      b = isempty(self.swigPtr);
    end
    function disp(self)
      disp(sprintf('<Swig object, ptr=%d>',self.swigPtr))
    end
    function varargout = subsref(self,s)
      if numel(s)==1
        switch s.type
          case '.'
            [varargout{1}] = builtin('subsref',self,substruct('.',s.subs,'()',{}));
          case '()'
            [varargout{1:nargout}] = builtin('subsref',self,substruct('.','paren','()',s.subs));
          case '{}'
            [varargout{1:nargout}] = builtin('subsref',self,substruct('.','brace','()',s.subs));
        end
      else
        [varargout{1:nargout}] = builtin('subsref',self,s);
      end
    end
    function self = subsasgn(self,s,v)
      if numel(s)==1
        switch s.type
          case '.'
            builtin('subsref',self,substruct('.',s.subs,'()',{v}));
          case '()'
            builtin('subsref',self,substruct('.','paren_asgn','()',{v, s.subs{:}}));
          case '{}'
            builtin('subsref',self,substruct('.','setbrace','()',{v, s.subs{:}}));
        end
      else
        self = builtin('subsasgn',self,s,v);
      end
    end
    function SwigSet(self,ptr)
        self.swigPtr = ptr;
    end
    function ptr = SwigGet(self)
        ptr = self.swigPtr;
    end
  end
end
