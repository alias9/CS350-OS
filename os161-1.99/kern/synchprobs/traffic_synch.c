#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */
// static struct semaphore *intersectionSem;

static volatile int inside;  //# threads at intersection
static volatile unsigned int direct;  //stores direction. start from the NORTH
static struct cv *condN, *condS, *condE, *condW;
static struct lock *lkDirect; 
static volatile unsigned int wait[4];

/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  /* replace this default implementation with your own implementation */

  // intersectionSem = sem_create("intersectionSem",1);
  // if (intersectionSem == NULL) {
  //   panic("could not create intersection semaphore");
  // }
  // return;

  lkDirect = lock_create("lock at intersection");
  if (lkDirect == NULL) {
    panic("could not create intersection lock");
  }
  condN = cv_create("cv north");
  if (condN == NULL) {
    panic("could not create north cv");
  }
  condS = cv_create("cv south");
  if (condS == NULL) {
    panic("could not create south cv");
  }
  condW = cv_create("cv west");
  if (condW == NULL) {
    panic("could not create west cv");
  }
  condE = cv_create("cv east");
  if (condE == NULL) {
    panic("could not create east cv");
  }
  wait[0] = 0;
  wait[1] = 0;
  wait[2] = 0;
  wait[3] = 0;
  inside = 0;
  direct = north;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  /* replace this default implementation with your own implementation */
  // KASSERT(intersectionSem != NULL);
  // sem_destroy(intersectionSem);

  KASSERT(lkDirect != NULL);
  KASSERT(condN != NULL);
  KASSERT(condS != NULL);
  KASSERT(condE != NULL);
  KASSERT(condW != NULL);

  cv_destroy(condN);
  cv_destroy(condS);
  cv_destroy(condE);
  cv_destroy(condW);
  lock_destroy(lkDirect);
  lkDirect = NULL;
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

//check if there's car from directions other than the origin waiting
static bool otherswaiting(Direction origin) {
  for (unsigned int i = 0; i < 4; ++i) {
    if (i != origin) {
      if(wait[i] > 0) return true;
    }
  }
  return false; 
}

void
intersection_before_entry(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  (void)origin;  /* avoid compiler complaint about unused parameter */
  (void)destination; /* avoid compiler complaint about unused parameter */
  // KASSERT(intersectionSem != NULL);
  // P(intersectionSem);

  KASSERT(lkDirect != NULL);
  KASSERT(condN != NULL);
  KASSERT(condS != NULL);
  KASSERT(condE != NULL);
  KASSERT(condW != NULL);
  // block threads from other directions
  lock_acquire(lkDirect);
  ++wait[origin];
  //if there's car waiting in different directions, let them go first
  if(otherswaiting(origin)) {
    switch(origin) {
      case north: 
        cv_wait(condN, lkDirect); break;
      case south: 
        cv_wait(condS, lkDirect); break;
      case east: 
        cv_wait(condE, lkDirect); break;
      case west: 
        cv_wait(condW, lkDirect); break;
    }
  }
  //if intersection is empty and no car waiting in other directions, 
  //let car from origin go
  else if (inside == 0) direct = origin;
  //origin not the same as current direction, wait.
  while (origin != direct) {
    switch(origin) {
      case north: 
        cv_wait(condN, lkDirect); break;
      case south: 
        cv_wait(condS, lkDirect); break;
      case east: 
        cv_wait(condE, lkDirect); break;
      case west: 
        cv_wait(condW, lkDirect); break;
    }
  }
  --wait[origin];
  ++inside;
  lock_release(lkDirect);
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  (void)origin;  /* avoid compiler complaint about unused parameter */
  (void)destination; /* avoid compiler complaint about unused parameter */
  // KASSERT(intersectionSem != NULL);
  // V(intersectionSem);

  KASSERT(lkDirect != NULL);
  KASSERT(condN != NULL);
  KASSERT(condS != NULL);
  KASSERT(condE != NULL);
  KASSERT(condW != NULL);

  lock_acquire(lkDirect);
  --inside;
  //intersection empty
  if (inside == 0) { 
    //change direction to a direction that there's a car waiting
    if(wait[(direct + 1) % 4] > 0) direct = (direct + 1) % 4;
    else if(wait[(direct + 2) % 4] > 0) direct = (direct + 2) % 4;
    else if(wait[(direct + 3) % 4] > 0) direct = (direct + 3) % 4;
      switch(direct) {
        case 0:
          cv_broadcast(condN, lkDirect); break;
        case 1: 
          cv_broadcast(condE, lkDirect); break;
        case 2: 
          cv_broadcast(condS, lkDirect); break;
        case 3: 
          cv_broadcast(condW, lkDirect); break;
      }
  }
  //intersection not empty. car from origin waiting, no car waiting in other directions
  else if (wait[origin] > 0 && !otherswaiting(origin)) {
    switch(origin) {
      case north:
        cv_broadcast(condN, lkDirect); break;
      case south: 
        cv_broadcast(condS, lkDirect); break;
      case east: 
        cv_broadcast(condE, lkDirect); break;
      case west: 
        cv_broadcast(condW, lkDirect); break;
    }
  }
  lock_release(lkDirect);
}
