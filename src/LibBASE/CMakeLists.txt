# === Base 静态库 ===
set(file_fold
filter
data
global
io
mutex
xml
coder
time
spdlog
set
)

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_header
    ${SRC_DIR}/LibBASE/header/${fold}/*.h)
    source_group("Header Files\\${fold}" FILES ${tmp_header})
    list(APPEND BASE_ALL_HEADER ${tmp_header}) 
    list(APPEND BASE_HEADER_DIR  ${SRC_DIR}/LibBASE/header/${fold})
endforeach()

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_source
     ${SRC_DIR}/LibBASE/source/${fold}/*.cpp)
    source_group("Source Files\\${fold}" FILES ${tmp_source})
    list(APPEND BASE_ALL_SRC ${tmp_source})
endforeach()

add_library(LibBASE STATIC ${BASE_ALL_SRC})
target_sources(LibBASE
PRIVATE
${BASE_ALL_HEADER}
)

target_include_directories(LibBASE PUBLIC
${BASE_HEADER_DIR}
${Third_zlib_ROOT}
${Third_Eigen_ROOT}
${Third_ceres_ROOT}/include/
${Third_PCL_ROOT}/Boost/include/boost-1_68/
${SRC_DIR}/LibSET/header/BASE
${SRC_DIR}/LibSET/header/GNSS
)
target_link_libraries(LibBASE
    PUBLIC
    ${Third_zlib_ROOT}/zlibd.lib
    ${Third_zlib_ROOT}/zlibstatic.lib
    ${Third_zlib_ROOT}/zlibstaticd.lib
    ${Third_ceres_ROOT}/debug/lib/ceres-debug.lib
    ${Third_ceres_ROOT}/debug/lib/gflagsd.lib
    ${Third_ceres_ROOT}/debug/lib/glog.lib
    ${Third_ceres_ROOT}/debug/lib/lapack.lib
    ${Third_ceres_ROOT}/debug/lib/libamd.lib
    ${Third_ceres_ROOT}/debug/lib/libbtf.lib
    ${Third_ceres_ROOT}/debug/lib/libcamd.lib
    ${Third_ceres_ROOT}/debug/lib/libccolamd.lib
    ${Third_ceres_ROOT}/debug/lib/libcholmod.lib
    ${Third_ceres_ROOT}/debug/lib/libcolamd.lib
    ${Third_ceres_ROOT}/debug/lib/libcxsparse.lib
    ${Third_ceres_ROOT}/debug/lib/libklu.lib
    ${Third_ceres_ROOT}/debug/lib/libldl.lib
    ${Third_ceres_ROOT}/debug/lib/libspqr.lib
    ${Third_ceres_ROOT}/debug/lib/libumfpack.lib
    ${Third_ceres_ROOT}/debug/lib/openblas.lib
    ${Third_ceres_ROOT}/debug/lib/suitesparseconfig.lib

    ${Third_PCL_ROOT}/Boost/lib/libboost_atomic-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_bzip2-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_chrono-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_container-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_context-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_contract-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_coroutine-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_date_time-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_exception-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_fiber-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_filesystem-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_graph-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_graph_parallel-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_iostreams-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_locale-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_log-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_log_setup-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_math_c99-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_math_c99f-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_math_c99l-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_math_tr1-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_math_tr1f-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_math_tr1l-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_mpi-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_numpy27-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_numpy37-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_prg_exec_monitor-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_program_options-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_python27-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_python37-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_random-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_regex-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_serialization-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_signals-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_stacktrace_noop-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_stacktrace_windbg-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_stacktrace_windbg_cached-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_system-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_test_exec_monitor-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_thread-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_timer-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_type_erasure-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_unit_test_framework-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_wave-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_wserialization-vc141-mt-x64-1_68.lib
    ${Third_PCL_ROOT}/Boost/lib/libboost_zlib-vc141-mt-x64-1_68.lib
)