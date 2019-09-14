#
# Copyright (C) Hiroyuki Sato. 2019.  ALL RIGHTS RESERVED.
# See file LICENSE for terms.
#

AC_ARG_WITH([progress64],
            [AC_HELP_STRING([--with-progress64=(DIR)],
                            [Enable the use of PROGRESS64 (default is autodetect).])
            ], [], [with_progress64=guess])


# PROGRESS64_PARSE_FLAGS(ARG, VAR_LIBS, VAR_LDFLAGS, VAR_CPPFLAGS)
# ----------------------------------------------------------
# Parse whitespace-separated ARG into appropriate LIBS, LDFLAGS, and
# CPPFLAGS variables.
AC_DEFUN([PROGRESS64_PARSE_FLAGS],
[for arg in $$1 ; do
    AS_CASE([$arg],
        [yes],               [],
        [no],                [],
        [-l*|*.a|*.so],      [$2="$$2 $arg"],
        [-L*|-WL*|-Wl*],     [$3="$$3 $arg"],
        [-I*],               [$4="$$4 $arg"],
        [*lib|*lib/|*lib64|*lib64/],[AS_IF([test -d $arg], [$3="$$3 -L$arg"],
                                 [AC_MSG_WARN([$arg of $1 not parsed])])],
        [*include|*include/],[AS_IF([test -d $arg], [$4="$$4 -I$arg"],
                                 [AC_MSG_WARN([$arg of $1 not parsed])])],
        [AC_MSG_WARN([$arg of $1 not parsed])])
done])

#
# Check for PROGRESS64 support
#
AS_IF([test "x$progress64_checked" != "xyes"],[

progress64_happy=no
AS_IF([test "x$with_progress64" != "xno"],
    [AS_CASE(["x$with_progress64"],
        [x|xguess|xyes],
            [AC_MSG_NOTICE([PROGRESS64 path was not specified. Guessing ...])
             with_progress64=/opt/progress64
             PROGRESS64_CPPFLAGS="-I$with_progress64/include/progress64 -I$with_progress64/include"
             PROGRESS64_LDFLAGS="-L$with_progress64/lib"
             PROGRESS64_LIBS="-lprogress64"],
        [x/*],
            [AC_MSG_NOTICE([PROGRESS64 path given as $with_progress64 ...])
             PROGRESS64_CPPFLAGS="-I$with_progress64/include"
             PROGRESS64_LDFLAGS="-L$with_progress64/lib -L$with_progress64"
             PROGRESS64_LIBS="-lprogress64"],
        [AC_MSG_NOTICE([PROGRESS64 flags given ...])
         PROGRESS64_PARSE_FLAGS([with_progress64],
                          [PROGRESS64_LIBS], [PROGRESS64_LDFLAGS], [PROGRESS64_CPPFLAGS])])

    SAVE_CPPFLAGS="$CPPFLAGS"
    SAVE_LDFLAGS="$LDFLAGS"
    SAVE_LIBS="$LIBS"
    CPPFLAGS="$PROGRESS64_CPPFLAGS $CPPFLAGS"
    LDFLAGS="$PROGRESS64_LDFLAGS $LDFLAGS"
    LIBS="$PROGRESS64_LIBS $LIBS"

    progress64_happy=yes
    AS_IF([test "x$progress64_happy" = xyes],
          [AC_CHECK_HEADERS([p64_spinlock.h], [progress64_happy=yes], [progress64_happy=no])])
    AS_IF([test "x$progress64_happy" = xyes],
          [AC_SEARCH_LIBS([p64_spinlock_init])
           AS_CASE(["x$ac_cv_search_p64_spinlock"],
               [xnone*], [],
               [xno], [progress64_happy=no],
               [x-l*], [PROGRESS64_LIBS="$ac_cv_search_p64_spinlock $PROGRESS64_LIBS"])])

    CPPFLAGS="$SAVE_CPPFLAGS"
    LDFLAGS="$SAVE_LDFLAGS"
    LIBS="$SAVE_LIBS"

    AS_IF([test "x$progress64_happy" = "xyes"],
          [AC_SUBST([PROGRESS64_CPPFLAGS])
           AC_SUBST([PROGRESS64_LDFLAGS])
           AC_SUBST([PROGRESS64_LIBS]),
           AC_DEFINE([HAVE_PROGRESS64], 1, [Enable PROGRESS64 support])],
          [AC_MSG_ERROR([PROGRESS64 not found])])
    ],
    [AC_MSG_WARN([PROGRESS64 was explicitly disabled])]
)

progress64_checked=yes
AM_CONDITIONAL([HAVE_PROGRESS64], [test "x$progress64_happy" != xno])

]) # AS_IF
