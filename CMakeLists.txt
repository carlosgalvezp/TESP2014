cmake_minimum_required(VERSION 2.8)

project(TESP2014)

file(GLOB SOURCES ./*.cpp)
file(GLOB HEADERS ./*.h)

add_executable(
  main
  ${SOURCES}
  ${HEADERS}
  )

target_link_libraries( 
  main 
  urglib2
  ncurses
  regmapReader
  porco_can_client
  porco_can_loco_client
  porco_can_subcrawler_client
  )

install(
  TARGETS main
  RUNTIME DESTINATION bin
  )
