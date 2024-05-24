include(CMakeFindDependencyMacro)

find_dependency(cnr_logger REQUIRED)
find_dependency(cnr_param REQUIRED)
find_dependency(cnr_class_loader REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/graph_coreTargets.cmake")
