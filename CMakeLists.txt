cmake_minimum_required(VERSION 3.15)
project(OPWKinematics VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(OPWKinematics
    src/OPWKinematics.cpp src/KinematicsConfig.cpp src/IKWindings.cpp src/IKSingularArm.cpp)
add_library(OPWKinematics::OPWKinematics ALIAS OPWKinematics)
target_compile_features(OPWKinematics PUBLIC cxx_std_17)
target_link_libraries(OPWKinematics PUBLIC Eigen3::Eigen)
target_include_directories(OPWKinematics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(OPWKinematics PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(opw_package_dir "${CMAKE_INSTALL_LIBDIR}/cmake/OPWKinematics")
configure_package_config_file(cmake/OPWKinematicsConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/OPWKinematicsConfig.cmake"
    INSTALL_DESTINATION "${opw_package_dir}")
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/OPWKinematicsConfigVersion.cmake"
    COMPATIBILITY SameMinorVersion)
install(TARGETS OPWKinematics EXPORT OPWKinematicsTargets)
install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(EXPORT OPWKinematicsTargets NAMESPACE OPWKinematics::
    DESTINATION "${opw_package_dir}")
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/OPWKinematicsConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/OPWKinematicsConfigVersion.cmake"
    DESTINATION "${opw_package_dir}")

option(OPW_BUILD_URDF_TOOL "Build the opw_from_urdf command-line tool" OFF)
if(OPW_BUILD_URDF_TOOL)
    find_package(urdfdom CONFIG REQUIRED)
    add_executable(opw_from_urdf tools/OPWFromURDF.cpp)
    target_link_libraries(opw_from_urdf PRIVATE OPWKinematics::OPWKinematics)
    if(TARGET urdfdom::urdfdom_model)
        target_link_libraries(opw_from_urdf PRIVATE urdfdom::urdfdom_model)
    else()
        target_link_libraries(opw_from_urdf PRIVATE urdfdom::urdfdom)
    endif()
    if(BUILD_SHARED_LIBS AND UNIX)
        file(RELATIVE_PATH opw_tool_lib_path "${CMAKE_INSTALL_FULL_BINDIR}" "${CMAKE_INSTALL_FULL_LIBDIR}")
        if(APPLE)
            set_target_properties(opw_from_urdf PROPERTIES INSTALL_RPATH "@loader_path/${opw_tool_lib_path}")
        else()
            set_target_properties(opw_from_urdf PROPERTIES INSTALL_RPATH "$ORIGIN/${opw_tool_lib_path}")
        endif()
    endif()
    install(TARGETS opw_from_urdf RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()

option(OPW_BUILD_PYTHON "Build the opw_kinematics Python module" OFF)
if(OPW_BUILD_PYTHON)
    if(CMAKE_VERSION VERSION_LESS 3.18)
        message(FATAL_ERROR "OPW_BUILD_PYTHON requires CMake 3.18 or newer")
    endif()
    set(Python3_FIND_IMPLEMENTATIONS CPython)
    find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter Development.Module)
    set(PYBIND11_FINDPYTHON ON)
    find_package(pybind11 CONFIG REQUIRED)
    if(pybind11_VERSION VERSION_LESS 2.12)
        message(FATAL_ERROR "OPW_BUILD_PYTHON requires pybind11 2.12 or newer")
    endif()
    Python3_add_library(opw_kinematics MODULE WITH_SOABI python/Bindings.cpp)
    target_include_directories(opw_kinematics PRIVATE src)
    target_link_libraries(opw_kinematics PRIVATE OPWKinematics::OPWKinematics pybind11::headers)
    set_target_properties(OPWKinematics PROPERTIES POSITION_INDEPENDENT_CODE ON)
    set_target_properties(opw_kinematics PROPERTIES
        CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
    set(opw_python_dir "${CMAKE_INSTALL_LIBDIR}/python")
    if(APPLE)
        set_target_properties(opw_kinematics PROPERTIES INSTALL_RPATH "@loader_path/..")
    elseif(UNIX)
        set_target_properties(opw_kinematics PROPERTIES INSTALL_RPATH "$ORIGIN/..")
    elseif(WIN32 AND BUILD_SHARED_LIBS)
        install(FILES $<TARGET_FILE:OPWKinematics> DESTINATION "${opw_python_dir}")
    endif()
    install(TARGETS opw_kinematics LIBRARY DESTINATION "${opw_python_dir}"
        RUNTIME DESTINATION "${opw_python_dir}")
endif()

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    include(CTest)
    if(BUILD_TESTING)
        add_executable(opw_regression tests/OPWKinematicsTest.cpp)
        target_link_libraries(opw_regression PRIVATE OPWKinematics::OPWKinematics)
        add_test(NAME opw_regression COMMAND opw_regression)
        if(OPW_BUILD_URDF_TOOL)
            add_test(NAME opw_urdf_round_trip COMMAND opw_from_urdf
                "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/opw_arm.urdf"
                --base base_link --tip tool0)
            add_test(NAME opw_urdf_diagnostics COMMAND "${CMAKE_COMMAND}"
                "-DOPW_TOOL=$<TARGET_FILE:opw_from_urdf>"
                "-DOPW_FIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/opw_arm.urdf"
                "-DOPW_TEST_DIR=${CMAKE_CURRENT_BINARY_DIR}"
                -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/URDFDiagnostics.cmake")
        endif()
        if(OPW_BUILD_PYTHON)
            set(opw_python_test_args "$<TARGET_FILE_DIR:opw_kinematics>")
            if(OPW_BUILD_URDF_TOOL)
                list(APPEND opw_python_test_args "$<TARGET_FILE:opw_from_urdf>"
                    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/opw_arm.urdf")
            endif()
            add_test(NAME opw_python_smoke COMMAND "${Python3_EXECUTABLE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/tests/PythonBindingsTest.py"
                ${opw_python_test_args})
        endif()
    endif()
endif()
