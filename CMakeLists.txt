# FILE(GLOB SRC *.cpp)

cmake_minimum_required(VERSION 2.6)

set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUNIX -D_LINUX=1 -DCMDLINE=1")

SET(SRC
  Xbus.cpp cppMTi3.cpp
)
ADD_LIBRARY(MTi3 SHARED ${SRC})
TARGET_LINK_LIBRARIES(MTi3 -lpigpiod_if2)
install(TARGETS MTi3 DESTINATION ${project_LIB_DIR}/)

SET(SRC
  cppReadMTi3.cpp
)
ADD_EXECUTABLE(readMTi3 ${SRC})
TARGET_LINK_LIBRARIES(readMTi3 -lpigpiod_if2 MTi3)

SET(SRC
  cppSetMeasurementOutputMTi3.cpp
)
ADD_EXECUTABLE(setMeasurementOutputMTi3 ${SRC})
TARGET_LINK_LIBRARIES(setMeasurementOutputMTi3 -lpigpiod_if2 MTi3)

SET(SRC
  cppSetMagLoggingOutputMTi3.cpp
)
ADD_EXECUTABLE(setMagLoggingOutputMTi3 ${SRC})
TARGET_LINK_LIBRARIES(setMagLoggingOutputMTi3 -lpigpiod_if2 MTi3)

SET(SRC
  cppPrintOutputConfigurationMTi3.cpp
)
ADD_EXECUTABLE(printOutputConfigurationMTi3 ${SRC})
TARGET_LINK_LIBRARIES(printOutputConfigurationMTi3 -lpigpiod_if2 MTi3)

SET(SRC
  cppReadLogMTi3.cpp
)
ADD_EXECUTABLE(readLogMTi3 ${SRC})
TARGET_LINK_LIBRARIES(readLogMTi3 -lpigpiod_if2 MTi3)