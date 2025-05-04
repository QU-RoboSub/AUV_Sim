#include <stdio.h>
#include <stdlib.h>
#include "Simulation.h"
#include "Simulation_private.h"
#include "rtwtypes.h"
#include "limits.h"
#include "rt_nonfinite.h"
#include "ext_work.h"
#include "linuxinitialize.h"
#define UNUSED(x)                      x = x
#define NAMELEN                        16

/* Function prototype declaration*/
void exitFcn(int sig);
void *terminateTask(void *arg);
void *baseRateTask(void *arg);
void *subrateTask(void *arg);
volatile boolean_T stopRequested = false;
volatile boolean_T runModel = true;
sem_t stopSem;
sem_t baserateTaskSem;
pthread_t schedulerThread;
pthread_t baseRateThread;
pthread_t backgroundThread;
void *threadJoinStatus;
int terminatingmodel = 0;
void *baseRateTask(void *arg)
{
  runModel = (rtmGetErrorStatus(Simulation_M) == (NULL)) && !rtmGetStopRequested
    (Simulation_M);
  while (runModel) {
    sem_wait(&baserateTaskSem);

    /* External mode */
    {
      boolean_T rtmStopReq = false;
      rtExtModePauseIfNeeded(Simulation_M->extModeInfo, 2, &rtmStopReq);
      if (rtmStopReq) {
        rtmSetStopRequested(Simulation_M, true);
      }

      if (rtmGetStopRequested(Simulation_M) == true) {
        rtmSetErrorStatus(Simulation_M, "Simulation finished");
        break;
      }
    }

    Simulation_step();

    /* Get model outputs here */
    rtExtModeCheckEndTrigger();
    stopRequested = !((rtmGetErrorStatus(Simulation_M) == (NULL)) &&
                      !rtmGetStopRequested(Simulation_M));
    runModel = !stopRequested;
  }

  runModel = 0;
  terminateTask(arg);
  pthread_exit((void *)0);
  return NULL;
}

void exitFcn(int sig)
{
  UNUSED(sig);
  rtmSetErrorStatus(Simulation_M, "stopping the model");
}

void *terminateTask(void *arg)
{
  UNUSED(arg);
  terminatingmodel = 1;

  {
    runModel = 0;

    /* Wait for background task to complete */
    CHECK_STATUS(pthread_join(backgroundThread, &threadJoinStatus), 0,
                 "pthread_join");
  }

  /* Terminate model */
  Simulation_terminate();
  rtExtModeShutdown(2);
  sem_post(&stopSem);
  return NULL;
}

void *backgroundTask(void *arg)
{
  while (runModel) {
    /* External mode */
    {
      boolean_T rtmStopReq = false;
      rtExtModeOneStep(Simulation_M->extModeInfo, 2, &rtmStopReq);
      if (rtmStopReq) {
        rtmSetStopRequested(Simulation_M, true);
      }
    }
  }

  pthread_exit((void *)0);
  return NULL;
}

int main(int argc, char **argv)
{
  rtmSetErrorStatus(Simulation_M, 0);
  rtExtModeParseArgs(argc, (const char_T **)argv, NULL);

  /* Initialize model */
  Simulation_initialize();

  /* External mode */
  rtSetTFinalForExtMode(&rtmGetTFinal(Simulation_M));
  rtExtModeCheckInit(2);

  {
    boolean_T rtmStopReq = false;
    rtExtModeWaitForStartPkt(Simulation_M->extModeInfo, 2, &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(Simulation_M, true);
    }
  }

  rtERTExtModeStartMsg();

  /* Call RTOS Initialization function */
  myRTOSInit(0.025, 0);

  /* Wait for stop semaphore */
  sem_wait(&stopSem);

#if (MW_NUMBER_TIMER_DRIVEN_TASKS > 0)

  {
    int i;
    for (i=0; i < MW_NUMBER_TIMER_DRIVEN_TASKS; i++) {
      CHECK_STATUS(sem_destroy(&timerTaskSem[i]), 0, "sem_destroy");
    }
  }

#endif

  return 0;
}
