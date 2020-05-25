include("${CMAKE_MODULE_PATH}/common_functions.cmake")

include(FindProtobuf)
FIND_PACKAGE(Protobuf REQUIRED)

MacroRequired(GENERATED_PROTO_DIR)
MacroRequired(SRCDIR)

include_directories(${GENERATED_PROTO_DIR})
include_directories(${SRCDIR}/thirdparty/protobuf-2.5.0/src)

add_definitions(-DPROTOBUF)

if( LINUXALL )
    set(PROTO_COMPILER "${SRCDIR}/devtools/bin/linux/protoc")
elseif( WINDOWS )
    set(PROTO_COMPILER "${SRCDIR}/devtools/bin/protoc.exe")
elseif( OSXALL )
    set(PROTO_COMPILER "${SRCDIR}/devtools/bin/osx32/protoc")
else()
    message(FATAL_ERROR "Platform Unknown!")
endif()

#Built a .proto file and add the resulting C++ to the target.
macro( TargetBuildAndAddProto TARGET_NAME PROTO_FILE PROTO_OUTPUT_FOLDER )
    set(PROTO_FILENAME)
    get_filename_component(PROTO_FILENAME ${PROTO_FILE} NAME_WLE) #name without any extensions (/home/gamer/swag.proto = swag)

    #TODO: check if .proto files need rebuild instead of rebuilding them each time.
    if( NO_REBUILD_PROTOS )
        MESSAGE(STATUS "NOT REBUILDING PROTOS!")
    else()
        message("Building proto file ${PROTO_FILE} to: ${PROTO_OUTPUT_FOLDER}")
        #invoke the protoc compiler. The proto_path order is important! see https://github.com/protocolbuffers/protobuf/issues/3183
        execute_process(COMMAND ${PROTO_COMPILER} --cpp_out=. --proto_path=${SRCDIR}/game/shared/cstrike15 --proto_path=${SRCDIR}/thirdparty/protobuf-2.5.0/src --proto_path=${SRCDIR}/gcsdk --proto_path=${SRCDIR}/game/shared --proto_path=${SRCDIR}/common ${PROTO_FILE}
                WORKING_DIRECTORY ${PROTO_OUTPUT_FOLDER}
                RESULT_VARIABLE PROTOBUF_RESULT
                OUTPUT_VARIABLE PROTOBUF_OUTPUT_VARIABLE)
        #add the generated .pb.cc to the target.
        message(STATUS "Proto(${PROTO_FILENAME}) - output(${PROTOBUF_OUTPUT_VARIABLE}) - result(${PROTOBUF_RESULT}) -file(${PROTO_OUTPUT_FOLDER}/${PROTO_FILENAME}.pb.cc)")
    endif()

    target_sources(${TARGET_NAME} PRIVATE ${PROTO_OUTPUT_FOLDER}/${PROTO_FILENAME}.pb.cc)
endmacro()

#THE NEWEST PROTOBUF WILL NOT WORK. SAVE THIS FOR LATER.
#$BuildAndAddProtoToTarget <TARGET_NAME> <PROTO_FILE/S> <PROTOS_OUT_DIR>
#This functionality is undocumented, read the source here
#https://github.com/protocolbuffers/protobuf/blob/master/cmake/protobuf-config.cmake.in
#macro( BuildAndAddProtoToTarget arg_TARGET_NAME arg_PROTO_FILES arg_PROTOS_OUT_DIR )
#    protobuf_generate(
#            LANGUAGE cpp
#            IMPORT_DIRS "${SRCDIR}/thirdparty/protobuf-2.5.0/src;${SRCDIR}/gcsdk;${SRCDIR}/game/shared;${SRCDIR}/game/shared/cstrike15;${SRCDIR}/common"
#            OUT_VAR PROTO_GENERATE_RESULT
#            PROTOS ${arg_PROTO_FILES}
#            PROTOC_OUT_DIR ${arg_PROTOS_OUT_DIR}
#    )
#    message("proto-generate result: ${PROTO_GENERATE_RESULT}")
#    #This smelly thing builds the protos in the proper directory, but adds the wrong directory to the target
#    #so we need to fix it.
#    foreach(file ${PROTO_GENERATE_RESULT})
#        set(PROTO_FILENAME)
#        get_filename_component(PROTO_FILENAME ${file} NAME)
#        message("Generated- ${PROTO_FILENAME}")
#        target_sources(${arg_TARGET_NAME} PRIVATE ${arg_PROTOS_OUT_DIR}/${PROTO_FILENAME})
#    endforeach()
#endmacro()