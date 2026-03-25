# === UWB 静态库 ===
set(file_fold
coder
data
process
set
)

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_header
    ${SRC_DIR}/LibUWB/header/${fold}/*.h)
    source_group("Header Files\\${fold}" FILES ${tmp_header})
    list(APPEND UWB_ALL_HEADER ${tmp_header}) 
    list(APPEND UWB_HEADER_DIR  ${SRC_DIR}/LibUWB/header/${fold})
endforeach()

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_source
     ${SRC_DIR}/LibUWB/source/${fold}/*.cpp)
    source_group("Source Files\\${fold}" FILES ${tmp_source})
    list(APPEND UWB_ALL_SRC ${tmp_source})
endforeach()

source_group("Header Files" FILES ${UWB_HDR})
source_group("Source Files" FILES ${UWB_SRC})
add_library(LibUWB STATIC ${UWB_ALL_SRC})

target_sources(LibUWB
PRIVATE
${UWB_ALL_HEADER}
)

target_link_libraries(LibUWB
PUBLIC  
LibBASE
${Third_ceres_ROOT}/lib/ceres.lib
${Third_ceres_ROOT}/lib/gflags.lib
${Third_ceres_ROOT}/lib/glog.lib
${Third_ceres_ROOT}/lib/lapack.lib
${Third_ceres_ROOT}/lib/libamd.lib
${Third_ceres_ROOT}/lib/libbtf.lib
${Third_ceres_ROOT}/lib/libcamd.lib
${Third_ceres_ROOT}/lib/libccolamd.lib
${Third_ceres_ROOT}/lib/libcholmod.lib
${Third_ceres_ROOT}/lib/libcolamd.lib
${Third_ceres_ROOT}/lib/libcxsparse.lib
${Third_ceres_ROOT}/lib/libklu.lib
${Third_ceres_ROOT}/lib/libldl.lib
${Third_ceres_ROOT}/lib/libspqr.lib
${Third_ceres_ROOT}/lib/libumfpack.lib
${Third_ceres_ROOT}/lib/openblas.lib
${Third_ceres_ROOT}/lib/suitesparseconfig.lib
)

target_include_directories(LibUWB 
PUBLIC
${UWB_HEADER_DIR}
${Third_PCL_ROOT}/Boost/include/boost-1_68/
${Third_ceres_ROOT}/include/
)

