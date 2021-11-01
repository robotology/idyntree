#
# simple Tokenizer for Matlab / Octave code
#

import re

class Token(object):
    NAME = 1
    OP = 2
    COMMENT = 3
    NL = 4
    WHITESPACE = 5

    def __init__(self, content, type):
        self.content = content
        self.type = type

    def __str__(self):
        return '"%s" [%s]' % (self.content, self.type)

    def __repr__(self):
        return self.__str__()

    @staticmethod
    def join(tks, sep=''):
        return sep.join(tk.content for tk in tks)

    @staticmethod
    def get_lines(tks, line_prefix=''):
        lines = []
        cur_line_parts = []

        def insert_prefix(line):
            return line_prefix + line

        for tk in tks:
            if tk.type == Token.NL:
                cur_line = insert_prefix(''.join(cur_line_parts))
                lines.append(cur_line)
                cur_line_parts = []
            else:
                cur_line_parts.append(tk.content)

        if len(cur_line_parts):
            cur_line = insert_prefix(''.join(cur_line_parts))
            lines.append(cur_line)

        return lines



class Tokenizer(object):
    '''
    Simple tokenizer of Matlab code

    Splits a string up in tokens representing a name, operator, comment,
    newline or comment.

    Current limitations:
    - no support for line continuations
    - floating point numbers may be represented by two name and one operator
    - multi-line comments are not recognized

    '''
    OP_CHARS = ['(', ')', '{', '}', '@', '^', '&', '*', '-', '+'
        , '=', ';', ':', '|', '<', '>', ',', ',/', '[', '\]', '.']

    @staticmethod
    def tokenize(string):
        lines = string.split('\n')

        tokens = []

        ws_sp = re.compile(r'(\s+)')

        op_re = ('([%s])' % ''.join(Tokenizer.OP_CHARS))
        op_sp = re.compile(op_re)

        for line in lines:
            inside_str = False
            comment_start = None
            for i, c in enumerate(line):
                if c == "'":
                    inside_str = not inside_str
                elif not inside_str and c == '%':
                    comment_start = i

            if comment_start is None:
                code_line = line
            else:
                code_line = line[:comment_start]

            for i, s in enumerate(ws_sp.split(code_line)):
                is_whitespace = i % 2 == 1
                if is_whitespace:
                    tk = Token(s, Token.WHITESPACE)
                    tokens.append(tk)
                else:
                    for j, t in enumerate(op_sp.split(s)):
                        is_op = j % 2 == 1
                        if len(t) == 0:
                            continue
                        type = Token.OP if is_op else Token.NAME
                        tk = Token(t, type)
                        tokens.append(tk)

            if comment_start is not None:
                comment = line[comment_start:]
                tk = Token(comment, Token.COMMENT)
                tokens.append(tk)

            tokens.append(Token('\n', Token.NL))

        return tokens

    @staticmethod
    def from_file(fn):
        with open(fn) as f:
            string = f.read()
            return Tokenizer.tokenize(string)



class TokenPattern(object):
    def __init__(self, content, type):
        self.content = content
        self.type = type

    def _matches(self, tk):
        raise NotImplementedError

    def __str__(self):
        return '%s(%s,%s)' % (self.__class__.__name__,
                              self.content,
                              self.type)

    @staticmethod
    def find(token_pats, tks, start=0, stop=None):
        n_pat = len(token_pats)
        n_tk = len(tks)

        for i_start in xrange(start, n_tk - n_pat):
            if stop is not None and i_start >= stop:
                break

            matches = True
            tk_pos = i_start

            for j, token_pat in enumerate(token_pats):
                tk_pos = token_pat._matches(tks, tk_pos)
                if tk_pos is None:
                    matches = False
                    break

            if matches:
                return (i_start, tk_pos)

        return None



class LiteralTokenPattern(TokenPattern):
    def _matches(self, tks, pos):
        tk = tks[pos]
        if (self.content is not None and
                    self.content != tk.content):
            return None

        if (self.type is not None and
                    self.type != tk.type):
            return None

        return pos + 1



class OptionalTypeTokenPattern(TokenPattern):
    def __init__(self, type):
        super(OptionalTypeTokenPattern, self).__init__(None, type)

    def _matches(self, tks, pos):
        for i in xrange(pos, len(tks)):
            tk = tks[i]
            same_type = self.type == tk.type

            if not same_type:
                break

        return i



class SkipUntilTokenPattern(TokenPattern):
    def _matches(self, tks, pos):
        n_tks = len(tks)
        for i in xrange(pos, n_tks):
            tk = tks[i]

            matches_content = self.content is None or tk.content == self.content
            matches_type = self.type == tk.type

            if matches_content and matches_type:
                return i

        return None



class SkipUntilTypeTokenPattern(SkipUntilTokenPattern):
    def __init__(self, type):
        super(SkipUntilTypeTokenPattern, self).__init__(None, type)

