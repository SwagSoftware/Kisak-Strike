#Cmake include for economy-related files
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
include(${CMAKE_MODULE_PATH}/common_functions.cmake)

MacroRequired( SRCDIR )

include_directories(${SRCDIR}/game/shared/econ)
include_directories(${SRCDIR}/gcsdk/steamextra)

#$Folder	"Economy"
#{
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/econ/econ_item_view.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/econ/econ_item_interface.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/econ/econ_item_system.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/econ/attribute_manager.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/econ/econ_entity.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/econ/econ_entity_creation.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/econ/econ_item_inventory.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/econ/econ_item.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/econ/econ_item_constants.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/econ/econ_item_schema.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/econ/item_selection_criteria.cpp")

    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/econ/localization_provider.cpp")
#}