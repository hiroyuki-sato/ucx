AC_CHECK_LIB([pthread], [pthread_create])
AC_CHECK_FUNC([pthread_spin_lock],
              AC_DEFINE([HAVE_PTHREAD_SPINLOCK], [1], [pthread spinlock is available]),
              [])

