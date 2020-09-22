#include "pthread_barrier.h"

int pthread_barrier_init(pthread_barrier_t *barrier, int *_dummy,
                         unsigned count)
{
    p64_barrier_init(barrier, count);
    return(0);
}


int pthread_barrier_wait(pthread_barrier_t *barrier)
{
    p64_barrier_wait(barrier);
    return(0);
}

int pthread_barrier_destroy(pthread_barrier_t *barrier)
{
    return(0);
}

int pthread_spin_init(pthread_spinlock_t *lock, int _dummy)
{
    p64_spinlock_init(lock);
    return(0);
}

int pthread_spin_lock(pthread_spinlock_t *lock)
{
    p64_spinlock_acquire(lock);
    return(0);
}

int pthread_spin_trylock(pthread_spinlock_t *lock)
{
    int status;
    status = p64_spinlock_try_acquire(lock);
    return(status);
}

int pthread_spin_unlock(pthread_spinlock_t *lock)
{
    p64_spinlock_release(lock);
}

int pthread_spin_destroy(pthread_spinlock_t *lock)
{
    return(0);
}
