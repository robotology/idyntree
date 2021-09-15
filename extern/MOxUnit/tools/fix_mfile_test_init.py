#!/usr/bin/python
#
# Fixes issue with recent (Matlab 2016 and later) test_suites defined using
# MOxUnit, which stopped working because function handles in scripts could no
# longer refer to sub-functions in a function calling that script.
#
# This function takes a filename of a Matlab .m file, and if necessary,
# can rewrite it. By default it will only show which changes would be applied;
# to apply the changes use the --apply option
#
# It can be used together with 'find' in bash, for example:
#
#      find ../tests/ -iname '*.m' | xargs -L1 ./fix_mfile_test_init.py --apply
#
# would look for tests defined in '../tests' and apply necessary changes

from matlab_tokenizer import Tokenizer, LiteralTokenPattern, \
    OptionalTypeTokenPattern, SkipUntilTypeTokenPattern, SkipUntilTokenPattern, \
    Token, TokenPattern

import argparse
import difflib
import logging
import sys

DIFF_SAME = '  '
DIFF_ADD = '+ '
DIFF_DELETE = '- '
DIFF_INFO = '? '



def get_fix_test_diff(tks):
    '''
    Compute diff necessary to

    Input
    -----
    tks: list
        list of matlab_lexer.Token objects

    Returns
    -------
    g: generator
        generator of diff objects
    '''
    Lit = LiteralTokenPattern
    OptType = OptionalTypeTokenPattern
    UntilType = SkipUntilTypeTokenPattern
    Until = SkipUntilTokenPattern
    T = Token

    # look for file with contents in the form of "function test_suite=foo"
    prefix = [UntilType(T.NAME),
              Lit('function', T.NAME),
              UntilType(T.NAME),
              Lit('test_suite', T.NAME),
              UntilType(T.NAME),
              Lit(None, T.NAME),
              UntilType(T.NL)]

    # localfunction_lines pattern (see below) that is going to be added if not
    # already in the file
    infix = [OptType(T.WHITESPACE),
             Lit('try', T.NAME),
             UntilType(T.NAME),
             Lit('test_functions', T.NAME),
             OptType(T.WHITESPACE),
             Lit('=', T.OP),
             Until('catch', T.NAME),
             Until('end', T.NAME),
             UntilType(T.NL),
             Lit(None, T.NL)]

    # part before which the localfunction_lines is inserted
    suffix = [Lit(None, T.NL),
              OptType(T.WHITESPACE),
              Lit('initTestSuite', T.NAME),
              UntilType(T.NL)]

    localfunction_lines = \
        [
            '    try % assignment of \'localfunctions\' is necessary in '
                                                        'Matlab >= 2016',
            '        test_functions=localfunctions();',
            '    catch % no problem; early Matlab versions can use '
                                                        'initTestSuite fine',
            '    end']

    pre_pos = TokenPattern.find(prefix, tks)
    if pre_pos is None:
        # does not match pattern, skip
        return None

    suf_pos = TokenPattern.find(suffix, tks, pre_pos[1])
    if suf_pos is None:
        # does not match pattern, skip
        return None

    # look for infix
    infix_pos = TokenPattern.find(infix, tks, pre_pos[1], suf_pos[0])
    has_infix = infix_pos is not None

    if has_infix:
        # file already contains usage of localfunctions
        infix_tks = tks[infix_pos[0]:infix_pos[1]]
        needs_replacement = Token.join(infix_tks) != localfunction_lines

        if needs_replacement:
            # no exact match, needs rewrite
            prefix_tks = tks[:infix_pos[0]]
            suffix_tks = tks[infix_pos[1]:]

            delta = list(difflib.ndiff(Token.get_lines(infix_tks, ''),
                                       localfunction_lines))

            def fix_hint(line):
                if line.startswith('?') and line.endswith('\n'):
                    line = line[:-2]
                return line

            delta = map(fix_hint, delta)

            diff_lines = [Token.get_lines(prefix_tks, DIFF_SAME),
                          delta,
                          Token.get_lines(suffix_tks, DIFF_SAME)]
        else:
            # no rewrite necessary
            diff_lines = Token.get_lines(tks, DIFF_SAME)
    else:
        # needs insert of use of localfunctions
        prefix_tks = tks[:suf_pos[0]]
        suffix_tks = tks[1 + suf_pos[0]:]

        localfunction_lines_with_prefix = [DIFF_ADD + line
                                           for line in localfunction_lines]

        diff_lines = [Token.get_lines(prefix_tks, DIFF_SAME),
                      localfunction_lines_with_prefix,
                      Token.get_lines(suffix_tks, DIFF_SAME)]


    # return a generator
    return (line for line in sum(diff_lines, []))



def content_changes(lines):
    return len(find_content_changes(lines)) > 0



def find_content_changes(lines):
    def is_diff_line(line):
        prefixes = [DIFF_DELETE, DIFF_ADD, DIFF_INFO]
        return any(line.startswith(prefix) for prefix in prefixes)

    return [i for i, line in enumerate(lines) if is_diff_line(line)]



def get_diff_summary(lines, context=5):
    line_idxs = find_content_changes(lines)
    line_candidates = set(i + c for i in line_idxs
                          for c in xrange(-context, context + 1))
    line_keep = list(i for i in line_candidates if i >= 0 and i < len(lines))
    line_keep.sort()

    return '\n'.join([lines[i] for i in line_keep])



def process_with_args(args):
    logging.basicConfig(format='%(message)s', level=args.loglevel)

    filename = args.filename
    if not filename.endswith('.m'):
        logging.error('File "%s" is not .m file, skip' % filename)
        sys.exit(1)

    tks = Tokenizer.from_file(filename)
    diff_generator = get_fix_test_diff(tks)
    if diff_generator is None:
        logging.error('File "%s" does not seem to define a test suite'
                      'for MOxUnit, skip' % filename)
        sys.exit(1)

    diff = list(diff_generator)

    if content_changes(diff):
        logging.info(
            'Changes for "%s"\n%s' % (filename, get_diff_summary(diff)))
        if args.apply:
            fixed_lines = list(difflib.restore(diff, 2))

            with open(filename, 'w') as f:
                f.write('\n'.join(fixed_lines))

            logging.info('File "%s" was rewritten' % filename)

        else:
            logging.info('Changes not applied; use --apply to rewrite "%s"' %
                         filename)

    else:
        logging.info('File "%s" does not require changes' % filename)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Fix MOxUnit test functions '
                                                 'to be compatible with recent '
                                                 ' (>=2016b) versions of '
                                                 'Matlab')
    parser.add_argument('-q', '--quiet',
                        help="Do not show output",
                        action="store_const", dest="loglevel",
                        const=logging.ERROR,
                        default=logging.INFO)
    parser.add_argument('--apply',
                        action='store_true',
                        help='apply changes')
    parser.add_argument('filename')

    args = parser.parse_args()
    process_with_args(args)
