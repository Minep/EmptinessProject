#pragma once

#define DEBUG_DRAWING 1
#define DEBUG_RIGID 1
#define DEBUG_RIGID_MOTION 1
#define DEBUG_RIGID_COLLISION 1
#define DEBUG_RIGID_MASS 0


#define SHOULD_DEBUG_RIGID DEBUG_DRAWING && DEBUG_RIGID
#define SHOULD_DEBUG_MOTION_TRACE DEBUG_DRAWING && DEBUG_RIGID && DEBUG_RIGID_MOTION
#define SHOULD_DEBUG_RIGID_COLLISION DEBUG_DRAWING && DEBUG_RIGID && DEBUG_RIGID_MOTION
#define SHOULD_DEBUG_RIGID_MASS DEBUG_DRAWING && DEBUG_RIGID && DEBUG_RIGID_MASS
