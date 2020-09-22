#ifndef PTHREAD_BARRIER_H
#define PTHREAD_BARRIER_H

#ifdef __APPLE__
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PTHREAD_BARRIER_SERIAL_THREAD (1)

typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int count;
    int nthread;
} pthread_barrier_t;

int pthread_barrier_init(pthread_barrier_t *barrier, int *_dummy,
                         unsigned count);

int pthread_barrier_wait(pthread_barrier_t *barrier);

int pthread_barrier_destroy(pthread_barrier_t *barrier);

#ifdef __cplusplus
}
#endif

#endif /* __APPLE__ */
#endif /* PTHREAD_BARRIER_H */
