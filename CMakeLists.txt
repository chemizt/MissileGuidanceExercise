cmake_minimum_required(VERSION 3.5)

project(MissileGuidanceExercise LANGUAGES CXX)

set(BIN_OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/out)

if(DEFINED CUSTOM_BIN_OUTPUT_ROOT)
    set(BIN_OUTPUT_ROOT ${CUSTOM_BIN_OUTPUT_ROOT})
endif()

message(BIN_OUTPUT_ROOT="${BIN_OUTPUT_ROOT}")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(UI ${CMAKE_SOURCE_DIR}/qml/mainwindow.ui)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets LinguistTools PrintSupport REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets LinguistTools PrintSupport REQUIRED)

set(TS_FILES locale/MissileGuidanceExercise_ru_RU.ts)

file(GLOB_RECURSE SOURCE_FILES source/**)
file(GLOB_RECURSE HEADER_FILES include/**)
file(GLOB_RECURSE RESOURCE_FILES *.*rc)

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    enable_language("RC")
endif()

set(PROJECT_SOURCES ${SOURCE_FILES} ${HEADER_FILES} ${TS_FILES} ${RESOURCE_FILES})

qt_wrap_ui(UI_HEADERS ${UI})

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(MGE64 WIN32 ${PROJECT_SOURCES} ${UI_HEADERS})
    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
else()
    add_executable(MGE64 WIN32 ${PROJECT_SOURCES} ${UI_HEADERS})
    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

target_include_directories(MGE64 PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(MGE64 PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::PrintSupport)
set_target_properties(MGE64 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_ROOT})
