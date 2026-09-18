idf_component_register(
    SRCS
        "src/attdet.c"
        "src/bits.c"
        "src/bwdet.c"
        "src/energy.c"
        "src/lc3.c"
        "src/ltpf.c"
        "src/mdct.c"
        "src/plc.c"
        "src/sns.c"
        "src/spec.c"
        "src/tables.c"
        "src/tns.c"
    INCLUDE_DIRS
        "include"
        "src"
    LDFRAGMENTS
        "linker.lf"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -O3 -ffast-math)
