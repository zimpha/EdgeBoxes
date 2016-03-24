cmake_minimum_required(VERSION 2.8)

set(EDGE_LIB_SRC convUtil.cpp EdgeDetector.cpp imPad.cpp rgbConvert.cpp EdgeBoxes.cpp gradientUtil.cpp imResample.cpp wrappers.cpp)

add_library(edge STATIC ${EDGE_LIB_SRC})
target_link_libraries(edge ${OpenCV_LIBS})
set_target_properties(edge PROPERTIES VERSION 1.0.0 SOVERSION 1)
