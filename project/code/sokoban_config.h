#ifndef SOKOBAN_CONFIG_H
#define SOKOBAN_CONFIG_H

#include <stdint.h>

/*
 * Storage/search capacity, not the box count of a level.
 * Every planner must use the count parsed from the current map.  The current
 * RT1064 RAM budget is validated for maps with up to ten boxes/targets.
 */
#define SOKOBAN_BOX_CAPACITY 10
#define SOKOBAN_BOX_MASK_BITS 16U
#define SOKOBAN_ELEMENT_TYPE_CAPACITY 26

typedef uint16_t SokobanBoxMask;

#define SOKOBAN_BOX_MASK_BIT(index_) \
    ((SokobanBoxMask)((SokobanBoxMask)1U << (uint8_t)(index_)))

static inline SokobanBoxMask sokoban_full_mask_u8(uint8_t count)
{
    if (count == 0U) return (SokobanBoxMask)0U;
    if (count >= SOKOBAN_BOX_MASK_BITS) return (SokobanBoxMask)0xFFFFU;
    return (SokobanBoxMask)(((uint32_t)1U << count) - 1U);
}

/* Competition rule, intentionally separate from box capacity. */
#define SOKOBAN_LEVEL3_BOMB_LIMIT 2
#define SOKOBAN_LEVEL3_BOMB_CAPACITY SOKOBAN_LEVEL3_BOMB_LIMIT

/* Box + target objects may all need recognition.  Exact subset DP remains
 * bounded; larger scenes switch to a scalable fixed-memory planner. */
#define SOKOBAN_RECOGNITION_OBJECT_CAPACITY (SOKOBAN_BOX_CAPACITY * 2)
#define SOKOBAN_RECOGNITION_EXACT_DP_MAX_OBJECTS 12

/* Fail at compile time if a future capacity change outruns the selected
 * bitmasks or the map label scheme.  The current count is still parsed from
 * each map at run time; these checks only guard storage capacity. */
#if SOKOBAN_BOX_CAPACITY > SOKOBAN_BOX_MASK_BITS
#error "SOKOBAN_BOX_CAPACITY exceeds SokobanBoxMask width"
#endif
#if SOKOBAN_BOX_CAPACITY > SOKOBAN_ELEMENT_TYPE_CAPACITY
#error "SOKOBAN_BOX_CAPACITY exceeds A..Z/a..z label capacity"
#endif
#if SOKOBAN_RECOGNITION_OBJECT_CAPACITY > 32
#error "Recognition object capacity exceeds uint32_t subset mask width"
#endif
#if SOKOBAN_RECOGNITION_EXACT_DP_MAX_OBJECTS > SOKOBAN_RECOGNITION_OBJECT_CAPACITY
#error "Exact recognition threshold exceeds recognition storage capacity"
#endif

#endif
