cmake_minimum_required( VERSION 3.6 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

# Create Project
project( sample )
add_executable( PCLFusion app.h app.cpp main.cpp util.h KinectFusionHelper.h KinectFusionHelper.cpp )

# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "PCLFusion" )

# Find Package
find_package( PCL 1.8 REQUIRED )

set( KinectSDK2_FUSION TRUE )
find_package( KinectSDK2 REQUIRED )

if( PCL_FOUND AND KinectSDK2_FOUND )
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )

  include_directories( ${KinectSDK2_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( ${PCL_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( PCLFusion ${PCL_LIBRARIES} )
  target_link_libraries( PCLFusion ${KinectSDK2_LIBRARIES} )

  # Post Build Event
  add_custom_command( TARGET PCLFusion POST_BUILD ${KinectSDK2_COMMANDS} )
endif()