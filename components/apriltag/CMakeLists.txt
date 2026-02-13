file(GLOB_RECURSE apriltag_sources *.c)

# Remove desktop-only files
list(FILTER apriltag_sources EXCLUDE REGEX "apriltag_pywrap.c")
list(FILTER apriltag_sources EXCLUDE REGEX "getopt.c")
list(FILTER apriltag_sources EXCLUDE REGEX ".*test.*")
list(FILTER apriltag_sources EXCLUDE REGEX ".*demo.*")

idf_component_register(
    SRCS ${apriltag_sources}
    INCLUDE_DIRS "."
)

target_compile_options(
    ${COMPONENT_LIB} 
    PRIVATE 
    -Wno-error 
    -Wno-error=incompatible-pointer-types 
    -Wno-error=char-subscripts
    -Wno-error=format
    -Wno-incompatible-pointer-types 
    -Wno-char-subscripts
    -Wno-format
    -Wno-shift-negative-value
)