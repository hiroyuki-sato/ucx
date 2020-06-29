/*
* Copyright (C) Mellanox Technologies Ltd. 2001-2014.  ALL RIGHTS RESERVED.
* Copyright (C) Huawei Technologies Co., Ltd. 2020. ALL RIGHTS RESERVED.
*
* See file LICENSE for terms.
*/

#ifndef UCS_SPINLOCK_H
#define UCS_SPINLOCK_H

#include <ucs/type/status.h>
#include <pthread.h>
#include <errno.h>

#ifdef HAVE_PROGRESS64
#include <p64_spinlock.h>
#endif

BEGIN_C_DECLS

/** @file spinlock.h */


/* Spinlock creation modifiers */
enum {
    UCS_SPINLOCK_FLAG_SHARED = UCS_BIT(0) /**< Make spinlock sharable in memory */
};

/**
 * Simple spinlock.
 */
typedef struct ucs_spinlock {
#ifdef HAVE_PROGRESS64
    p64_spinlock_t     lock;
#else
    pthread_spinlock_t lock;
#endif
} ucs_spinlock_t;

/**
 * Reentrant spinlock.
 */
typedef struct ucs_recursive_spinlock {
    ucs_spinlock_t super;
    int            count;
    pthread_t      owner;
} ucs_recursive_spinlock_t;

#define UCS_SPINLOCK_OWNER_NULL ((pthread_t)-1)


static ucs_status_t ucs_spinlock_init(ucs_spinlock_t *lock, int flags)
{
#ifndef HAVE_PROGRESS64
    int ret, lock_flags;
#endif

#ifdef HAVE_PROGRESS64
    p64_spinlock_init(&lock->lock);
#else
    if (flags & UCS_SPINLOCK_FLAG_SHARED) {
        lock_flags = PTHREAD_PROCESS_SHARED;
    } else {
        lock_flags = PTHREAD_PROCESS_PRIVATE;
    }

    ret = pthread_spin_init(&lock->lock, lock_flags);
    if (ret != 0) {
        return UCS_ERR_IO_ERROR;
    }
#endif
    return UCS_OK;
}

static inline ucs_status_t
ucs_recursive_spinlock_init(ucs_recursive_spinlock_t* lock, int flags)
{
    lock->count = 0;
    lock->owner = UCS_SPINLOCK_OWNER_NULL;

    return ucs_spinlock_init(&lock->super, flags);
}

static inline ucs_status_t ucs_spinlock_destroy(ucs_spinlock_t *lock)
{
#ifndef HAVE_PROGRESS64
    int ret;

    ret = pthread_spin_destroy(&lock->lock);
    if (ret != 0) {
        if (errno == EBUSY) {
            return UCS_ERR_BUSY;
        } else {
            return UCS_ERR_INVALID_PARAM;
        }
    }
#endif

    return UCS_OK;
}

static inline ucs_status_t
ucs_recursive_spinlock_destroy(ucs_recursive_spinlock_t *lock)
{
    if (lock->count != 0) {
        return UCS_ERR_BUSY;
    }

    return ucs_spinlock_destroy(&lock->super);
}

static inline int
ucs_recursive_spin_is_owner(ucs_recursive_spinlock_t *lock, pthread_t self)
{
    return lock->owner == self;
}

static inline void ucs_spin_lock(ucs_spinlock_t *lock)
{
#ifdef HAVE_PROGRESS64
    p64_spinlock_try_acquire(&lock->lock);
#else
    pthread_spin_lock(&lock->lock);
#endif
}

static inline void ucs_recursive_spin_lock(ucs_recursive_spinlock_t *lock)
{
    pthread_t self = pthread_self();

    if (ucs_recursive_spin_is_owner(lock, self)) {
        ++lock->count;
        return;
    }

    ucs_spin_lock(&lock->super);
    lock->owner = self;
    ++lock->count;
}

static inline int ucs_spin_try_lock(ucs_spinlock_t *lock)
{
#ifdef HAVE_PROGRESS64
    if (p64_spinlock_try_acquire(&lock->lock) != true) {
        return 0;
    }
#else
    if (pthread_spin_trylock(&lock->lock) != 0) {
        return 0;
    }
#endif

    return 1;
}

static inline int ucs_recursive_spin_trylock(ucs_recursive_spinlock_t *lock)
{
    pthread_t self = pthread_self();

    if (ucs_recursive_spin_is_owner(lock, self)) {
        ++lock->count;
        return 1;
    }

    if (ucs_spin_try_lock(&lock->super) == 0) {
        return 0;
    }

    lock->owner = self;
    ++lock->count;
    return 1;
}

static inline void ucs_spin_unlock(ucs_spinlock_t *lock)
{
#ifdef HAVE_PROGRESS64
    p64_spinlock_release(&lock->lock);
#else
    pthread_spin_unlock(&lock->lock);
#endif
}

static inline void ucs_recursive_spin_unlock(ucs_recursive_spinlock_t *lock)
{
    --lock->count;
    if (lock->count == 0) {
        lock->owner = UCS_SPINLOCK_OWNER_NULL;
        ucs_spin_unlock(&lock->super);
    }
}

END_C_DECLS

#endif
