#ifndef PTHREAD_BARRIER_H
#define PTHREAD_BARRIER_H

#include <p64_barrier.h>
#include <p64_spinlock.h>

BEGIN_C_DECLS

typedef p64_barrier_t  pthread_barrier_t;
typedef p64_spinlock_t pthread_spinlock_t;

int pthread_barrier_init(pthread_barrier_t *barrier, int *_dummy,
                         unsigned count);

int pthread_barrier_wait(pthread_barrier_t *barrier);

int pthread_barrier_destroy(pthread_barrier_t *barrier);

int pthread_spin_init(pthread_spinlock_t *lock, int _dummy);

int pthread_spin_lock(pthread_spinlock_t *lock);

int pthread_spin_trylock(pthread_spinlock_t *lock);

int pthread_spin_unlock(pthread_spinlock_t *lock);

int pthread_spin_destroy(pthread_spinlock_t *lock);

END_C_DECLS

#endif

