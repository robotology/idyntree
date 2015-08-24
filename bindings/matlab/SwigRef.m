classdef SwigRef < handle
  properties 
    swigInd
  end
  methods
    function disp(self)
      disp(sprintf('<Swig object, ind=%d>',self.swigInd))
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
            builtin('subsref',self,substruct('.','setparen','()',{v, s.subs{:}}));
          case '{}'
            builtin('subsref',self,substruct('.','setbrace','()',{v, s.subs{:}}));
        end
      else
        self = builtin('subsasgn',self,s,v);
      end
    end
  end
end
