cmake_minimum_required(VERSION 3.5)

project(MissileGuidanceExercise LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/out/bin/debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/out/bin/release)

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

set(PROJECT_SOURCES ${SOURCE_FILES} ${HEADER_FILES} ${TS_FILES})

qt5_wrap_ui(UI_HEADERS ${UI})

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(MissileGuidanceExercise WIN32 ${PROJECT_SOURCES} ${UI_HEADERS})
    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
else()
    add_executable(MissileGuidanceExercise WIN32 ${PROJECT_SOURCES} ${UI_HEADERS})
    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

target_include_directories(MissileGuidanceExercise PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(MissileGuidanceExercise PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::PrintSupport)
