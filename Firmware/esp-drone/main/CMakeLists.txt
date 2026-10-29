idf_component_register(SRCS "main.c" "mender_ota.c"
                       INCLUDE_DIRS "."
                       LDFRAGMENTS linker_fragment.lf)