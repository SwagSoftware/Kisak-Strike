set(libprotobuf_files
  ${protobuf_source_dir}/src/google/protobuf/stubs/strutil.cc
  ${protobuf_source_dir}/src/google/protobuf/stubs/strutil.h
  ${protobuf_source_dir}/src/google/protobuf/stubs/substitute.cc
  ${protobuf_source_dir}/src/google/protobuf/stubs/substitute.h
  ${protobuf_source_dir}/src/google/protobuf/stubs/structurally_valid.cc
  ${protobuf_source_dir}/src/google/protobuf/descriptor.cc
  ${protobuf_source_dir}/src/google/protobuf/descriptor.pb.cc
  ${protobuf_source_dir}/src/google/protobuf/descriptor_database.cc
  ${protobuf_source_dir}/src/google/protobuf/dynamic_message.cc
  ${protobuf_source_dir}/src/google/protobuf/extension_set_heavy.cc
  ${protobuf_source_dir}/src/google/protobuf/generated_message_reflection.cc
  ${protobuf_source_dir}/src/google/protobuf/message.cc
  ${protobuf_source_dir}/src/google/protobuf/reflection_ops.cc
  ${protobuf_source_dir}/src/google/protobuf/service.cc
  ${protobuf_source_dir}/src/google/protobuf/text_format.cc
  ${protobuf_source_dir}/src/google/protobuf/unknown_field_set.cc
  ${protobuf_source_dir}/src/google/protobuf/wire_format.cc
  ${protobuf_source_dir}/src/google/protobuf/io/gzip_stream.cc
  ${protobuf_source_dir}/src/google/protobuf/io/printer.cc
  ${protobuf_source_dir}/src/google/protobuf/io/tokenizer.cc
  ${protobuf_source_dir}/src/google/protobuf/io/zero_copy_stream_impl.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/importer.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/parser.cc
)

#kisak-strike: add this or the descriptor database is global!
add_definitions(-fvisibility=hidden -fpic)

add_library(libprotobuf ${protobuf_SHARED_OR_STATIC}
  ${libprotobuf_lite_files} ${libprotobuf_files})
target_link_libraries(libprotobuf ${CMAKE_THREAD_LIBS_INIT} ${ZLIB_LIBRARIES})
target_include_directories(libprotobuf PUBLIC ${protobuf_source_dir}/src)
if(MSVC AND protobuf_BUILD_SHARED_LIBS)
  target_compile_definitions(libprotobuf
    PUBLIC  PROTOBUF_USE_DLLS
    PRIVATE LIBPROTOBUF_EXPORTS)
endif()
set_target_properties(libprotobuf PROPERTIES
    OUTPUT_NAME ${LIB_PREFIX}protobuf
    DEBUG_POSTFIX "${protobuf_DEBUG_POSTFIX}")
