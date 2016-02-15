#  iDynTree 
#  Copyright: (C) 2015 iCub Facility, Istituto Italiano di Tecnologia
#  Authors: Silvio Traversaro <silvio.traversaro@iit.it>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

# TinyXML
add_subdirectory(tinyxml)
set(TinyXML_INCLUDE_DIRS ${TinyXML_INCLUDE_DIRS} PARENT_SCOPE)
set(TinyXML_LIBRARIES ${TinyXML_LIBRARIES} PARENT_SCOPE)
set(TinyXML_DEFINITIONS ${TinyXML_DEFINITIONS} PARENT_SCOPE)

