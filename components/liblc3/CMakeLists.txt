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

# LC3plus / LC3plus HR off unless enabled in Kconfig (PUBLIC: lc3.h sizes structs with these macros)
if(CONFIG_LC3_ENABLE_PLUS_MODES)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC LC3_PLUS=1 LC3_PLUS_HR=1)
else()
    target_compile_definitions(${COMPONENT_LIB} PUBLIC LC3_PLUS=0 LC3_PLUS_HR=0)
endif()
