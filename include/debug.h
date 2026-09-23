//
// Created by James Gallagher on 6/7/20.
//

#ifndef SOIL_SENSOR_DEBUG_H
#define SOIL_SENSOR_DEBUG_H

// IO(x): compiled in only when DEBUG is non-zero. Historically routed setup()-time
// diagnostics to Serial; leaf_node.cc now uses it only inside setup() (FR-008).
#if DEBUG
#define IO(x) \
    do {      \
        x;    \
    } while (0)
#else
#define IO(x)
#endif

// IO_LOG(x): compiled in only when DEBUG_LOG is non-zero. Wraps calls to
// debug_log() so a non-debug build elides both the call and its arguments (FR-009).
#if DEBUG_LOG
#define IO_LOG(x) \
    do {          \
        x;        \
    } while (0)
#else
#define IO_LOG(x)
#endif

#endif //SOIL_SENSOR_DEBUG_H
