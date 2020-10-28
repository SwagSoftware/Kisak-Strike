set(libprotoc_files
  ${protobuf_source_dir}/src/google/protobuf/compiler/code_generator.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/command_line_interface.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/plugin.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/plugin.pb.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/subprocess.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/subprocess.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/zip_writer.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/zip_writer.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_enum.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_enum.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_enum_field.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_enum_field.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_extension.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_extension.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_field.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_field.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_file.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_file.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_generator.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_helpers.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_helpers.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_message.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_message.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_message_field.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_message_field.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_options.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_primitive_field.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_primitive_field.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_service.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_service.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_string_field.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/cpp/cpp_string_field.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_enum.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_enum.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_enum_field.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_enum_field.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_extension.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_extension.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_field.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_field.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_file.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_file.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_generator.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_helpers.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_helpers.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_message.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_message.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_message_field.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_message_field.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_primitive_field.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_primitive_field.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_service.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_service.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_string_field.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_string_field.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_doc_comment.cc
  ${protobuf_source_dir}/src/google/protobuf/compiler/java/java_doc_comment.h
  ${protobuf_source_dir}/src/google/protobuf/compiler/python/python_generator.cc
)

#kisak-strike: add this or the descriptor database is global!
add_definitions(-fvisibility=hidden -fpic)

add_library(libprotoc ${protobuf_SHARED_OR_STATIC}
  ${libprotoc_files})
target_link_libraries(libprotoc libprotobuf)
if(MSVC AND protobuf_BUILD_SHARED_LIBS)
  target_compile_definitions(libprotoc
    PUBLIC  PROTOBUF_USE_DLLS
    PRIVATE LIBPROTOC_EXPORTS)
endif()
set_target_properties(libprotoc PROPERTIES
    COMPILE_DEFINITIONS LIBPROTOC_EXPORTS
    OUTPUT_NAME ${LIB_PREFIX}protoc
    DEBUG_POSTFIX "${protobuf_DEBUG_POSTFIX}")
