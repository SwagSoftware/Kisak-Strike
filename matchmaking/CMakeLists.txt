#CMake for all matchmaking targets.
cmake_minimum_required(VERSION 3.17)
project(matchmaking_libraries)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
include(${CMAKE_MODULE_PATH}/detect_platform.cmake)

#Normal MatchmakingBase
set(OUTLIBNAME "matchmakingbase")
set(MM_DS "0")

include( ./matchmaking_base_inc.cmake )
target_compile_definitions(${OUTLIBNAME} PRIVATE -DMATCHMAKING_DLL)

#Dedicated Server MatchmakingBase
set(OUTLIBNAME "matchmakingbase_ds")
set(MM_DS "1")

include( ./matchmaking_base_inc.cmake )
target_compile_definitions(${OUTLIBNAME} PRIVATE -DMATCHMAKING_DS_DLL -DSWDS)

#CSGO Matchmaking
set(OUTBINNAME "matchmaking_client")

include( ./matchmaking_cstrike15_inc.cmake )
target_compile_definitions(${OUTBINNAME} PRIVATE -DMATCHMAKING_DLL)
target_link_libraries(${OUTBINNAME} matchmakingbase)
target_link_libraries(${OUTBINNAME} tier2_client mathlib_client)

#CSGO Matchmaking DS
set(OUTBINNAME "matchmaking_ds_client")

include( ./matchmaking_cstrike15_inc.cmake )
target_compile_definitions(${OUTBINNAME} PRIVATE -DMATCHMAKING_DS_DLL -DSWDS)
target_link_libraries(${OUTBINNAME} matchmakingbase_ds)
target_link_libraries(${OUTBINNAME} tier2_client mathlib_client)

