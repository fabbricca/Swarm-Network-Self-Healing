cmake_minimum_required(VERSION 3.16)
project(swarm_simulator LANGUAGES CXX)

# ns-3 (3.46+) headers use C++20 library features (e.g. remove_cvref_t).
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(swarm_demo_sim1
	apps/help_proxy_sim.cpp
	platform/ns3/base_station/ns3_base_station.cpp
	modules/controller/controller.cpp
	modules/communication/communication_manager.cpp
	modules/dispatch/dispatch_manager.cpp
	platform/ns3/drone/ns3_drone.cpp
	modules/flood/flood_manager.cpp
	modules/neighbor/neighbor_manager.cpp
	modules/neighbor/neighbor_info.cpp
	platform/ns3/custom_mobility/custom_mobility.cpp
	platform/ns3/position/ns3_position.cpp
	platform/ns3/velocity_actuator/ns3_velocity_actuator.cpp
	modules/uwb_ranging/uwb_ranging_manager.cpp
	platform/ns3/uwb_anchor/ns3_uwb_anchor.cpp
	platform/ns3/uwb_channel/uwb_channel.cpp
	platform/ns3/uwb_transport/uwb_transport.cpp
)

add_executable(controller_tuner
	apps/controller_tuner.cpp
)

target_include_directories(swarm_demo_sim1 PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/..
)

# Prefer ns-3 CMake package config (works with modern ns-3 installs).
find_package(ns3 CONFIG QUIET)

if(ns3_FOUND)
	message(STATUS "Found ns-3 via CMake package")
		target_link_libraries(swarm_demo_sim1 PRIVATE
			ns3::core
			ns3::network
			ns3::mobility
			ns3::netanim
		)
else()
	message(STATUS "ns-3 CMake package not found; trying pkg-config")
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(NS3DEV QUIET IMPORTED_TARGET ns3-dev)
	if(NS3DEV_FOUND)
		target_link_libraries(swarm_demo_sim1 PRIVATE PkgConfig::NS3DEV)
	else()
		pkg_check_modules(NS3 REQUIRED IMPORTED_TARGET ns3)
		target_link_libraries(swarm_demo_sim1 PRIVATE PkgConfig::NS3)
	endif()
endif()

