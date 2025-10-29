# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
        ../src/room_detector.cpp
        ../src/ransac_line_detector.cpp
        ../src/hungarian.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
        ../src/room_detector.h
        ../src/ransac_line_detector.h
        ../src/hungarian.h
)

SET (LIBS ${LIBS}  tbb)
LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-fmax-errors=1 -fno-char8_t)

