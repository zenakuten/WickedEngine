cmake_minimum_required(VERSION 3.19)
set (SOURCE_FILES
	Tests.cpp
	Tests.h
)

if (WIN32)
	list (APPEND SOURCE_FILES
		main_Windows.cpp
		main_Windows.h
		Tests.rc
	)

	add_executable(Tests WIN32 ${SOURCE_FILES})

	target_link_libraries(Tests PUBLIC
		WickedEngine_Windows
	)

	set(LIB_DXCOMPILER "dxcompiler.dll")
else()
	list (APPEND SOURCE_FILES
		main_${PLATFORM}.cpp
	)

	add_executable(Tests ${SOURCE_FILES})

	target_link_libraries(Tests PUBLIC
		WickedEngine
	)

	set(LIB_DXCOMPILER "libdxcompiler.so")
endif ()

if(WICKED_ENABLE_IPO)
set_target_properties(Tests PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION ON
    INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF
)
endif()

target_precompile_headers(Tests PRIVATE stdafx.h)

add_executable(CustomShaderLifecycleTest
	CustomShaderLifecycleTest.cpp
)
target_link_libraries(CustomShaderLifecycleTest PRIVATE WickedEngine)
add_test(NAME CustomShaderLifecycleTest COMMAND CustomShaderLifecycleTest)
set_target_properties(CustomShaderLifecycleTest PROPERTIES FOLDER "Tests")

add_executable(PhysicsContractTest PhysicsContractTest.cpp)
target_link_libraries(PhysicsContractTest PRIVATE WickedEngine)
add_test(NAME PhysicsContractTest COMMAND PhysicsContractTest)
set_tests_properties(PhysicsContractTest PROPERTIES TIMEOUT 120)
set_target_properties(PhysicsContractTest PROPERTIES FOLDER "Tests")

add_executable(EditorViewportTest EditorViewportTest.cpp ../../Editor/Translator.cpp)
target_link_libraries(EditorViewportTest PRIVATE WickedEngine)
add_test(NAME EditorViewportTest COMMAND EditorViewportTest)
set_target_properties(EditorViewportTest PROPERTIES FOLDER "Tests")

add_executable(CSGTest CSGTest.cpp ../../WickedEngine/wiCSG.cpp)
add_test(NAME CSGTest COMMAND CSGTest)
set_tests_properties(CSGTest PROPERTIES TIMEOUT 30)
set_target_properties(CSGTest PROPERTIES FOLDER "Tests")

add_executable(CSGSnapTest CSGSnapTest.cpp ../../WickedEngine/wiCSG.cpp)
add_test(NAME CSGSnapTest COMMAND CSGSnapTest)
set_tests_properties(CSGSnapTest PROPERTIES TIMEOUT 30)
set_target_properties(CSGSnapTest PROPERTIES FOLDER "Tests")

add_executable(CSGSceneTest CSGSceneTest.cpp)
target_link_libraries(CSGSceneTest PRIVATE WickedEngine)
add_test(NAME CSGSceneTest COMMAND CSGSceneTest)
set_tests_properties(CSGSceneTest PROPERTIES TIMEOUT 30)
set_target_properties(CSGSceneTest PROPERTIES FOLDER "Tests")


if (MSVC)
	set_property(TARGET Tests PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endif ()

# Copy content to build folder:
add_dependencies(Tests Content)

add_custom_command(
	TARGET Tests POST_BUILD
	# Copy the shader library next to the executable
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${WICKED_ROOT_DIR}/WickedEngine/${LIB_DXCOMPILER} ${CMAKE_CURRENT_BINARY_DIR}
	# Copy the resources
	COMMAND ${CMAKE_COMMAND} -E ${COPY_OR_SYMLINK_DIR_CMD} ${CMAKE_CURRENT_SOURCE_DIR}/images ${CMAKE_CURRENT_BINARY_DIR}/images
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/test_script.lua ${CMAKE_CURRENT_BINARY_DIR}/test_script.lua
	# Copy font test resources
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/yumin.ttf ${CMAKE_CURRENT_BINARY_DIR}/

	COMMENT "$<IF:$<BOOL:${WICKED_ENABLE_SYMLINKS}>,Symlinking,Copying> Tests resources"
	VERBATIM
)
