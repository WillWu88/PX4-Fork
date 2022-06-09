px4_add_library(LQRControl
	LQRControl.cpp
	LQRControl.hpp
)
target_compile_options(LQRControl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(LQRControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(LQRControl PRIVATE mathlib eigen)

px4_add_unit_gtest(SRC LQRControlTest.cpp LINKLIBS LQRControl)
