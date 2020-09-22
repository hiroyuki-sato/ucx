/*
 * Copyright (C) 2020 by Keisuke Fukuda & Hiroyuki Sato, All rights reserved.
 */

#include <pthread.h>
#include <errno.h>

#ifdef __APPLE__

#include "pthread_barrier.h"

#if EINVAL == PTHREAD_BARRIER_SERIAL_THREAD || PTHREAD_BARRIER_SERIAL_THREAD == 0
# error "Wrong definition of PTHREAD_BARRIER_SERIAL_THREAD"
#endif

int pthread_barrier_init(pthread_barrier_t *barrier, int *_dummy,
                         unsigned count)
{
    int ret;
    
    if (count == 0) {
        return EINVAL;
    }
    
    ret = pthread_mutex_init(&barrier->mutex, NULL);
    if (ret < 0) {
        return ret;
    }

    ret = pthread_cond_init(&barrier->cond, NULL);
    if (ret < 0) {
        pthread_mutex_destroy(&barrier->mutex);
        return ret;
    }
    return 0;
}

int pthread_barrier_wait(pthread_barrier_t *barrier)
{
    pthread_mutex_lock(&barrier->mutex);
    barrier->count++;

    if (barrier->count == barrier->nthread) {
        barrier->count = 0;
        pthread_cond_broadcast(&barrier->cond);
        pthread_mutex_unlock(&barrier->mutex);
        return PTHREAD_BARRIER_SERIAL_THREAD;
    } else {
        pthread_cond_wait(&barrier->cond, &barrier->mutex);
        pthread_mutex_unlock(&barrier->mutex);
        return 0;
    }
}

int pthread_barrier_destroy(pthread_barrier_t *barrier) {
    pthread_cond_destroy(&barrier->cond);
    pthread_mutex_destroy(&barrier->mutex);
    barrier->nthread = 0;
    barrier->count = 0;
    return 0;
}

#endif
