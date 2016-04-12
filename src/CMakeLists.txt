cmake_minimum_required(VERSION 2.8)

set(GRAD_LIB_SRC wrappers.cpp gradientUtil.cpp CellArray.cpp CellArray.h)
add_library(grad STATIC ${GRAD_LIB_SRC})
target_link_libraries(grad ${OpenCV_LIBS})
set_target_properties(grad PROPERTIES VERSION 1.0.0 SOVERSION 1)

set(IMAGE_LIB_SRC imPad.cpp imPadMex.h rgbConvert.cpp rgbConvertMex.h imResample.cpp imResampleMex.h wrappers.cpp CellArray.cpp CellArray.h)
add_library(image STATIC ${IMAGE_LIB_SRC})
target_link_libraries(image ${OpenCV_LIBS})
set_target_properties(image PROPERTIES VERSION 1.0.0 SOVERSION 1)

set(CONV_LIB_SRC wrappers.cpp convUtil.cpp convConstMex.h CellArray.cpp CellArray.h)
add_library(conv STATIC ${CONV_LIB_SRC})
target_link_libraries(conv ${OpenCV_LIBS})
set_target_properties(conv PROPERTIES VERSION 1.0.0 SOVERSION 1)

set(EDGE_LIB_SRC ${GRAD_LIB_SRC} ${IMAGE_LIB_SRC} ${CONV_LIB_SRC} EdgeDetector.cpp EdgeBoxes.cpp box.cpp)
add_library(edge STATIC ${EDGE_LIB_SRC})
target_link_libraries(edge ${OpenCV_LIBS})
set_target_properties(edge PROPERTIES VERSION 1.0.0 SOVERSION 1)

set(CHANNEL_LIB_SRC ${GRAD_LIB_SRC} ${IMAGE_LIB_SRC} ${CONV_LIB_SRC} chnsCompute.cpp chnsPyramid.cpp)
add_library(channel STATIC ${CHANNEL_LIB_SRC})
target_link_libraries(channel ${OpenCV_LIBS})
set_target_properties(channel PROPERTIES VERSION 1.0.0 SOVERSION 1)

set(ACF_LIB_SRC ${GRAD_LIB_SRC} ${IMAGE_LIB_SRC} ${CONV_LIB_SRC} ${CHANNEL_LIB_SRC} ACFDetector.cpp box.cpp bbNms.cpp)
add_library(acf STATIC ${ACF_LIB_SRC})
target_link_libraries(acf ${OpenCV_LIBS})
set_target_properties(acf PROPERTIES VERSION 1.0.0 SOVERSION 1)
