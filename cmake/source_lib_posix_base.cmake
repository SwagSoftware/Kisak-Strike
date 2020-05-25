include("${CMAKE_MODULE_PATH}/common_functions.cmake")

set(IS_LIB_PROJECT "1")
include("${CMAKE_MODULE_PATH}/source_posix_base.cmake")

#set(ConfigurationType "Static Library (.lib)") #not used
MacroRequired(OUTLIBNAME)

#Target
add_library(${OUTLIBNAME} STATIC)
message("Adding static-library target: (${OUTLIBNAME})")

set_target_properties(${OUTLIBNAME} PROPERTIES OUTPUT_NAME "${OUTLIBNAME}")
set_target_properties(${OUTLIBNAME} PROPERTIES SUFFIX "${_STATICLIB_EXT}")
set_target_properties(${OUTLIBNAME} PROPERTIES PREFIX "")

set_target_properties( ${OUTLIBNAME} PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY "${OUTLIBDIR}"
        LIBRARY_OUTPUT_DIRECTORY "${OUTLIBDIR}"
        RUNTIME_OUTPUT_DIRECTORY "${OUTLIBDIR}"
        )

set(OutputFile "${OUTLIBDIR}/${OUTLIBNAME}${_STATICLIB_EXT}")