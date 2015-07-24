macro(add_rtf_test testfile)
    add_test(NAME ${testfile} COMMAND   testrunner --verbose --test ${CMAKE_CURRENT_SOURCE_DIR}/${testfile})
endmacro()

if(IDYNTREE_USES_PYTHON)
    add_rtf_test(geometry.py)
endif()