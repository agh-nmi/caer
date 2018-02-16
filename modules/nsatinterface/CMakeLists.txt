IF (NOT NSATINTERFACE)
  SET(NSATINTERFACE 0 CACHE BOOL "Enable the spike to dynapse stream")
ENDIF()

IF (NSATINTERFACE)
    ADD_LIBRARY(nsatinterface SHARED nsatinterface.c)

    TARGET_LINK_LIBRARIES(nsatinterface ${CAER_C_LIBS} curses)
ENDIF()
