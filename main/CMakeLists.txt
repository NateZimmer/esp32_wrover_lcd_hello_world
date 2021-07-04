set(srcs "lcd/lcd_lib.c" "lcd/lcd_fonts.c" "main.c")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")
