#pragma once lol
if(DEFINED common_functions_cmake)
    return() #return exits entire file
endif()
set(common_functions_cmake "defined")

#$MacroRequired <Macro> [DefaultValue]
macro (MacroRequired MacroName)
    set(ExtraMacroArgs ${ARGN})
    list(LENGTH ExtraMacroArgs NumExtraMacroArgs)
    if(NOT DEFINED ${MacroName})
        if(NumExtraMacroArgs GREATER 0)
            set(${MacroName} ${ARGV1})
            message(STATUS "Using Default value ${ARGV1} for RequiredMacro ${MacroName}")
        else()
            message(FATAL_ERROR "Required Macro ${MacroName} does not exist!")
        endif()
    endif()
endmacro()