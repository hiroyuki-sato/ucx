/**
 * Copyright (C) Hiroyuki Sato. 2019. ALL RIGHTS RESERVED.
 *
 * See file LICENSE for terms.
 */

#include "event_set.h"

#include <ucs/debug/memtrack.h>
#include <ucs/debug/log.h>
#include <ucs/debug/assert.h>
#include <ucs/sys/math.h>
#include <ucs/sys/compiler.h>

#include <string.h>
#include <errno.h>
#include <unistd.h>
#ifdef HAVE_SYS_EPOLL_H
#include <sys/epoll.h>
#else
#include <sys/event.h>
#endif

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif


enum {
    UCS_SYS_EVENT_SET_EXTERNAL_EVENT_FD = UCS_BIT(0),
};

struct ucs_sys_event_set {
    int epfd;
    int flags;
};


const unsigned ucs_sys_event_set_max_wait_events =
    UCS_ALLOCA_MAX_SIZE / sizeof(struct epoll_event);


static inline int ucs_event_set_map_to_raw_events(int events)
{
    int raw_events = 0;
    int ev_read,ev_write,ev_error,ev_et;

#ifdef HAVE_SYS_EPOLL_H
    ev_read  = EPOLLIN;
    ev_write = EPOLLOUT;
    ev_error = EPOLLERR;
    ev_et    = EPOLLEET;
#else
    ev_read  = EVFILT_READ;
    ev_write = EVFILT_WRITE;
    ev_error = -1; /* TODO */
    ev_et    = -1; /* TODO */
#endif

    if (events & UCS_EVENT_SET_EVREAD) {
         raw_events |= ev_read;
    }
    if (events & UCS_EVENT_SET_EVWRITE) {
         raw_events |= ev_error;
    }
    if (events & UCS_EVENT_SET_EVERR) {
         raw_events |= ev_error;
    }
    if (events & UCS_EVENT_SET_EVET) {
        raw_events  |= ev_et;
    }
    return raw_events;
}

static inline int ucs_event_set_map_to_events(int raw_events)
{
    int events = 0;

#ifdef HAVE_SYS_EPOLL_H
    if (raw_events & EPOLLIN) {
         events |= UCS_EVENT_SET_EVREAD;
    }
    if (raw_events & EPOLLOUT) {
         events |= UCS_EVENT_SET_EVWRITE;
    }
    if (raw_events & EPOLLERR) {
         events |= UCS_EVENT_SET_EVERR;
    }
    if (raw_events & EPOLLET) {
        events  |= UCS_EVENT_SET_EVET;
    }
#else
    if (raw_events & EVFILT_READ) {
         events |= UCS_EVENT_SET_EVREAD;
    }
    if (raw_events & EVFILT_WRITE) {
         events |= UCS_EVENT_SET_EVWRITE;
    }
/* TODO
    if (raw_events & EPOLLERR) {
         events |= UCS_EVENT_SET_EVERR;
    }
    if (raw_events & EPOLLET) {
        events  |= UCS_EVENT_SET_EVET;
    }
 */
#endif
    return events;
}

ucs_status_t
ucs_event_set_create(ucs_sys_event_set_t **event_set_p, int user_fd)
{
    ucs_sys_event_set_t *event_set;
    ucs_status_t status;

    event_set = ucs_malloc(sizeof(ucs_sys_event_set_t), "ucs_sys_event_set");
    if (event_set == NULL) {
        ucs_error("unable to allocate memory ucs_sys_event_set_t object");
        status = UCS_ERR_NO_MEMORY;
        goto out_create;
    }

    event_set->epfd  = user_fd;
    event_set->flags = 0;

    if (event_set->epfd == -1) {
#ifdef HAVE_SYS_EPOLL_H
        /* Create epoll set the thread will wait on */
        event_set->epfd = epoll_create(1);
        if (event_set->epfd < 0) {
            ucs_error("epoll_create() failed: %m");
            status = UCS_ERR_IO_ERROR;
            goto err_free;
        }
#else
        event_set->epfd = kqueue();
        if ( event_set->epfd == -1 ) {
            ucs_error("kqueue() failed: %m");
            status = UCS_ERR_IO_ERROR;
            goto err_free;
        }
#endif
    } else {
        event_set->flags |= UCS_SYS_EVENT_SET_EXTERNAL_EVENT_FD;
    }

    *event_set_p = event_set;
    return UCS_OK;

err_free:
    ucs_free(event_set);
out_create:
    return status;
}

ucs_status_t ucs_event_set_add(ucs_sys_event_set_t *event_set, int event_fd,
                               ucs_event_set_type_t events, void *callback_data)
{
#ifdef HAVE_SYS_EPOLL_H
    struct epoll_event raw_event;
#else
    struct kevent kq_event;
    int kq_filter;
#endif
    int ret;

#ifdef HAVE_SYS_EPOLL_H
    memset(&raw_event, 0, sizeof(raw_event));
    raw_event.events   = ucs_event_set_map_to_raw_events(events);
    raw_event.data.ptr = callback_data;

    ret = epoll_ctl(event_set->epfd, EPOLL_CTL_ADD, event_fd, &raw_event);
    if (ret < 0) {
        ucs_error("epoll_ctl(epfd=%d, ADD, fd=%d) failed: %m", event_set->epfd,
                  event_fd);
        return UCS_ERR_IO_ERROR;
    }
#else
    memset(&kqev, 0, sizeof(kq_event));
    kq_filter = ucs_event_set_map_to_raw_events(events);
    /* TODO */
    EV_SET(&kq_event, event_fd, kq_filter, EV_MOD, 0, 0, NULL);
    ret = kevent(event_set->epfd, &kqev, 1, NULL, 0, NULL);
    

#endif
    return UCS_OK;
}

ucs_status_t ucs_event_set_mod(ucs_sys_event_set_t *event_set, int event_fd,
                               ucs_event_set_type_t events, void *callback_data)
{
    struct epoll_event raw_event;
    int ret;

    memset(&raw_event, 0, sizeof(raw_event));
    raw_event.events   = ucs_event_set_map_to_raw_events(events);
    raw_event.data.ptr = callback_data;

    ret = epoll_ctl(event_set->epfd, EPOLL_CTL_MOD, event_fd, &raw_event);
    if (ret < 0) {
        ucs_error("epoll_ctl(epfd=%d, MOD, fd=%d) failed: %m", event_set->epfd,
                  event_fd);
        return UCS_ERR_IO_ERROR;
    }

    return UCS_OK;
}

ucs_status_t ucs_event_set_del(ucs_sys_event_set_t *event_set, int event_fd)
{
    int ret;

    ret = epoll_ctl(event_set->epfd, EPOLL_CTL_DEL, event_fd, NULL);
    if (ret < 0) {
        ucs_error("epoll_ctl(epfd=%d, DEL, fd=%d) failed: %m", event_set->epfd,
                  event_fd);
        return UCS_ERR_IO_ERROR;
    }

    return UCS_OK;
}

ucs_status_t ucs_event_set_wait(ucs_sys_event_set_t *event_set,
                                unsigned *num_events, int timeout_ms,
                                ucs_event_set_handler_t event_set_handler,
                                void *arg)
{
    struct epoll_event *events;
    int nready, i, io_events;

    ucs_assert(event_set_handler != NULL);
    ucs_assert(num_events != NULL);
    ucs_assert(*num_events <= ucs_sys_event_set_max_wait_events);

    events = ucs_alloca(sizeof(*events) * *num_events);

    nready = epoll_wait(event_set->epfd, events, *num_events, timeout_ms);
    if (ucs_unlikely(nready < 0)) {
        *num_events = 0;
        if (errno == EINTR) {
            return UCS_INPROGRESS;
        }
        ucs_error("epoll_wait() failed: %m");
        return UCS_ERR_IO_ERROR;
    }

    ucs_assert(nready <= *num_events);
    ucs_trace_poll("epoll_wait(epfd=%d, num_events=%u, timeout=%d) returned %u",
                   event_set->epfd, *num_events, timeout_ms, nready);

    for (i = 0; i < nready; i++) {
        io_events = ucs_event_set_map_to_events(events[i].events);
        event_set_handler(events[i].data.ptr, io_events, arg);
    }

    *num_events = nready;
    return UCS_OK;
}

void ucs_event_set_cleanup(ucs_sys_event_set_t *event_set)
{
    if (!(event_set->flags & UCS_SYS_EVENT_SET_EXTERNAL_EVENT_FD)) {
        close(event_set->epfd);
    }
    ucs_free(event_set);
}

ucs_status_t ucs_event_set_fd_get(ucs_sys_event_set_t *event_set, int *fd_p)
{
    if (event_set == NULL) {
         return UCS_ERR_INVALID_PARAM;
    }

    *fd_p = event_set->epfd;
    return UCS_OK;
}
