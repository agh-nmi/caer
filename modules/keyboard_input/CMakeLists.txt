IF (NOT KEYBIN)
	SET(KEYBIN 0 CACHE BOOL "Enable keyboard input")
ENDIF()

IF (KEYBIN)
    ADD_LIBRARY(keyboard_input SHARED keyboard_input.c)

    TARGET_LINK_LIBRARIES(keyboard_input ${CAER_C_LIBS} curses)
ENDIF()
