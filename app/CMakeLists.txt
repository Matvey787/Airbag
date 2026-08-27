cmake_minimum_required(VERSION 3.16)
project(airbag_gui VERSION 1.0.0 LANGUAGES CXX)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Charts LinguistTools)
# Add Location/Positioning/Quick for native map (MapLibre via Qt Location)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Charts LinguistTools Network QuickWidgets Quick3D )
find_package(nlohmann_json 3.2.0 REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(TS_FILES app_ru_RU.ts)
set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        ${TS_FILES}
        third_party/QRangeSlider/cpp/QRangeSlider.cpp
)


set(FIRMWARE_PARAMS_REL_PATH "../../../firmwares/esp32_firmware/data/firmware_vars.json")
set(MAP_DIR_REL_PATH "../../map/")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/project_info.h.cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/project_info.h"
    @ONLY
)

qt_add_executable(airbag_gui
    MANUAL_FINALIZATION
    ${PROJECT_SOURCES}
)
qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})

file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)
file(GLOB_RECURSE APP_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
)

list(REMOVE_ITEM APP_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/mainwindow.cpp"
)
list(REMOVE_ITEM APP_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/mainwindow.h"
)

list(FILTER APP_SOURCES EXCLUDE REGEX "${CMAKE_BINARY_DIR}")
list(FILTER APP_HEADERS EXCLUDE REGEX "${CMAKE_BINARY_DIR}")

target_sources(airbag_gui
    PRIVATE
        ${APP_SOURCES}
        ${APP_HEADERS}
)

target_include_directories(airbag_gui
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)


target_link_libraries(airbag_gui 
    PRIVATE 
        Qt${QT_VERSION_MAJOR}::Widgets 
        Qt${QT_VERSION_MAJOR}::Charts
        Qt${QT_VERSION_MAJOR}::Network
        Qt${QT_VERSION_MAJOR}::QuickWidgets
        Qt${QT_VERSION_MAJOR}::Quick3D
        nlohmann_json::nlohmann_json
)
target_include_directories(airbag_gui
    PRIVATE
        third_party/QRangeSlider/cpp/
)

if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.airbag_gui)
endif()
set_target_properties(airbag_gui PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)
include(GNUInstallDirs)
install(TARGETS airbag_gui
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(airbag_gui)
endif()

