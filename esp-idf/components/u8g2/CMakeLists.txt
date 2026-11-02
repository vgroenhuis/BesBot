# u8g2 (github.com/olikraus/u8g2, csrc at d6c8499): monochrome display library with many fonts. Only the fonts in
# csrc/u8g2_fonts_used.c are included (the full font file is 38 MB).
file(GLOB U8G2_SRCS "${CMAKE_CURRENT_LIST_DIR}/csrc/*.c")
idf_component_register(SRCS ${U8G2_SRCS} INCLUDE_DIRS "csrc")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable -Wno-unused-variable -Wno-unused-but-set-variable)
