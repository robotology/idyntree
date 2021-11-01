.PHONY: help \
        install-matlab install-octave install \
        uninstall-matlab uninstall-octave uninstall \
        test-matlab test-octave test

MATLAB?=matlab
OCTAVE?=octave

TESTDIR=$(CURDIR)/tests
ROOTDIR=$(CURDIR)/MOxUnit
UTILDIR=$(ROOTDIR)/util

ADDPATH=orig_dir=pwd();cd('$(ROOTDIR)');moxunit_set_path();cd(orig_dir)
RMPATH=rmpath('$(ROOTDIR)');
SAVEPATH=savepath();exit(0)

INSTALL=$(ADDPATH);$(SAVEPATH)
UNINSTALL=$(RMPATH);$(SAVEPATH)

help:
	@echo "Usage: make <target>, where <target> is one of:"
	@echo "------------------------------------------------------------------"
	@echo "  install            to add MOxUnit to the Matlab and GNU Octave"
	@echo "                     search paths, using whichever is present"
	@echo "  uninstall          to remove MOxUnit from the Matlab and GNU"
	@echo "                     Octave search paths, using whichever is"
	@echo "                     present"
	@echo "  test               to run tests using the Matlab and GNU Octave"
	@echo "                     search paths, whichever is present"
	@echo ""
	@echo "  install-matlab     to add MOxUnit to the Matlab search path"
	@echo "  install-octave     to add MOxUnit to the GNU Octave search path"
	@echo "  uninstall-matlab   to remove MOxUnit from the Matlab search path"
	@echo "  uninstall-octave   to remove MOxUnit from the GNU Octave search"
	@echo "                     path"
	@echo "  test-matlab        to run tests using Matlab"
	@echo "  test-octave        to run tests using GNU Octave"
	@echo "------------------------------------------------------------------"
	@echo ""
	@echo "Environmental variables:"
	@echo "  WITH_COVERAGE      Enable line coverage registration"
	@echo "  JUNIT_XML_FILE     Rest results XML output"
	@echo "  COVER              Directory to compute line coverage for"
	@echo "  COVER_XML_FILE    	Coverage XML output filename	"
	@echo "  COVER_JSON_FILE    Coverage JSON output filename"
	@echo "  COVER_HTML_DIR     Coverage HTML output directory"
	@echo "  COVER_HTML_DIR     Coverage HTML output directory"
	@echo "  RUN_DOC_TEST       If set, run documentation tests"
	@echo ""

RUNTESTS_ARGS?='-verbose'
TEST_RUNNER?=moxunit_runtests

ifdef RUN_DOC_TEST
	RUNTESTS_ARGS+=,'${ROOTDIR}','${UTILDIR}'
	TEST_RUNNER=modox_runtests
else
	RUNTESTS_ARGS+=,'${TESTDIR}'
	ifdef JUNIT_XML_FILE
		RUNTESTS_ARGS +=,'-junit_xml_file','$(JUNIT_XML_FILE)'
	endif

	ifdef WITH_COVERAGE
		ifndef COVER
			#$(error COVER variable must be set when using WITH_COVERAGE)
		endif
		RUNTESTS_ARGS+=,'-with_coverage','-cover','$(COVER)'
		export COVER

		ifdef COVER_XML_FILE
			 RUNTESTS_ARGS+=,'-cover_xml_file','$(COVER_XML_FILE)'
			 export COVER_XML_FILE
		endif

		ifdef COVER_HTML_DIR
			 RUNTESTS_ARGS+=,'-cover_html_dir','$(COVER_HTML_DIR)'
			 export COVER_HTML_DIR
		endif

		ifdef COVER_JSON_FILE
			 RUNTESTS_ARGS+=,'-cover_json_file','$(COVER_JSON_FILE)'
			 export COVER_JSON_FILE
		endif

		ifdef JUNIT_XML_FILE
			 RUNTESTS_ARGS+=,'-junit_xml_file','$(JUNIT_XML_FILE)'
			 export JUNIT_XML_FILE
		endif
	endif
	TEST_RUNNER=moxunit_runtests
endif

TEST=$(ADDPATH);success=$(TEST_RUNNER)($(RUNTESTS_ARGS));exit(~success);

MATLAB_BIN=$(shell which $(MATLAB))
OCTAVE_BIN=$(shell which $(OCTAVE))

ifeq ($(MATLAB_BIN),)
	# for Apple OSX, try to locate Matlab elsewhere if not found
    MATLAB_BIN=$(shell ls /Applications/MATLAB_R20*/bin/${MATLAB} 2>/dev/null | tail -1)
endif
	
MATLAB_RUN=$(MATLAB_BIN) -nojvm -nodisplay -nosplash -r
OCTAVE_RUN=$(OCTAVE_BIN) --no-gui --quiet --eval

install-matlab:
	@if [ -n "$(MATLAB_BIN)" ]; then \
		$(MATLAB_RUN) "$(INSTALL)"; \
	else \
		echo "matlab binary could not be found, skipping"; \
	fi;

install-octave:
	@if [ -n "$(OCTAVE_BIN)" ]; then \
		$(OCTAVE_RUN) "$(INSTALL)"; \
	else \
		echo "octave binary could not be found, skipping"; \
	fi;

install:
	@if [ -z "$(MATLAB_BIN)$(OCTAVE_BIN)" ]; then \
		@echo "Neither matlab binary nor octave binary could be found" \
		exit 1; \
	fi;
	$(MAKE) install-matlab
	$(MAKE) install-octave
	

uninstall-matlab:
	@if [ -n "$(MATLAB_BIN)" ]; then \
		$(MATLAB_RUN) "$(UNINSTALL)"; \
	else \
		echo "matlab binary could not be found, skipping"; \
	fi;
	
uninstall-octave:
	@if [ -n "$(OCTAVE_BIN)" ]; then \
		$(OCTAVE_RUN) "$(UNINSTALL)"; \
	else \
		echo "octave binary could not be found, skipping"; \
	fi;
	
uninstall:
	@if [ -z "$(MATLAB_BIN)$(OCTAVE_BIN)" ]; then \
		@echo "Neither matlab binary nor octave binary could be found" \
		exit 1; \
	fi;
	$(MAKE) uninstall-matlab
	$(MAKE) uninstall-octave


test-matlab:
	@if [ -n "$(MATLAB_BIN)" ]; then \
		$(MATLAB_RUN) "$(TEST)"; \
	else \
		echo "matlab binary could not be found, skipping"; \
	fi;

test-octave:
	if [ -n "$(OCTAVE_BIN)" ]; then \
		$(OCTAVE_RUN) "$(TEST)"; \
	else \
		echo "octave binary could not be found, skipping"; \
	fi;

test:
	@if [ -z "$(MATLAB_BIN)$(OCTAVE_BIN)" ]; then \
		@echo "Neither matlab binary nor octave binary could be found" \
		exit 1; \
	fi;
	$(MAKE) test-matlab
	$(MAKE) test-octave



