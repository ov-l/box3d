# Benchmark app

set(BOX3D_BENCHMARK_FILES
	main.c
)

add_executable(benchmark ${BOX3D_BENCHMARK_FILES})
add_executable(mover_cache_benchmark mover_cache_analysis.c)

set_target_properties(benchmark PROPERTIES C_STANDARD 17)
set_target_properties(mover_cache_benchmark PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)

target_link_libraries(benchmark PRIVATE box3d shared)
target_link_libraries(mover_cache_benchmark PRIVATE box3d)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "" FILES ${BOX3D_BENCHMARK_FILES})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "" FILES mover_cache_analysis.c)
