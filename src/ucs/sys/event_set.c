/**
 * Copyright (C) Hiroyuki Sato. 2019. ALL RIGHTS RESERVED.
 *
 * See file LICENSE for terms.
 */

#include "event_set.h"

#include <ucs/debug/memtrack.h>
#include <ucs/debug/log.h>
#include <ucs/debug/assert.h>

#ifdef HAVE_SYS_EPOLL_H
#include <sys/epoll.h>
#else
#include <sys/event.h>
#endif
#include <string.h>
#include <errno.h>
#include <unistd.h>

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#define UCS_EVENT_EPOLL_MAX_EVENTS 16

struct ucs_sys_event_set {
    int epfd;
};


static inline int ucs_event_set_map_to_raw_events(int events)
{
    int raw_events = 0;
    int read, write;
#ifdef HAVE_SYS_EPOLL_H
    read = EPOLLIN;
    write = EPOLLOUT;
#else
    read = EVFILT_READ;
    write = EVFILT_WRITE;
#endif

    if (events & UCS_EVENT_SET_EVREAD) {
         raw_events |= read;
    }
    if (events & UCS_EVENT_SET_EVWRITE) {
         raw_events |= write;
    }
    return raw_events;
}

static inline int ucs_event_set_map_to_events(int raw_events)
{
    int events = 0;
    int read, write;
#ifdef HAVE_SYS_EPOLL_H
    read = EPOLLIN;
    write = EPOLLOUT;
#else
    read = EVFILT_READ;
    write = EVFILT_WRITE;
#endif

    if (raw_events & read) {
         events |= UCS_EVENT_SET_EVREAD;
    }
    if (raw_events & write) {
         events |= UCS_EVENT_SET_EVWRITE;
    }
    return events;
}

ucs_status_t ucs_event_set_create(ucs_sys_event_set_t **event_set_p)
{
    ucs_sys_event_set_t *event_set;
    ucs_status_t status;

    event_set = ucs_malloc(sizeof(ucs_sys_event_set_t), "ucs_sys_event_set");
    if (event_set == NULL) {
        status = UCS_ERR_NO_MEMORY;
        goto out_create;
    }

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

    *event_set_p = event_set;
    return UCS_OK;

err_free:
    ucs_free(event_set);
out_create:
    return status;
}

ucs_status_t ucs_event_set_add(ucs_sys_event_set_t *event_set, int event_fd,
                               ucs_event_set_type_t events)
{
#ifdef HAVE_SYS_EPOLL_H
    struct epoll_event raw_event;
#else
    struct kevent kqev;
    int kq_filter;
#endif
    int ret;

#ifdef HAVE_SYS_EPOLL_H
    memset(&raw_event, 0, sizeof(raw_event));
    raw_event.events   = ucs_event_set_map_to_raw_events(events);
    raw_event.data.fd  = event_fd;

    ret = epoll_ctl(event_set->epfd, EPOLL_CTL_ADD, event_fd, &raw_event);
    if (ret < 0) {
        ucs_error("epoll_ctl(epfd=%d, ADD, fd=%d) failed: %m", event_set->epfd,
                  event_fd);
        return UCS_ERR_IO_ERROR;
    }
#else
    memset(&kqev, 0, sizeof(kqev));
    kq_filter = ucs_event_set_map_to_raw_events(events);
    EV_SET(&kqev, event_fd, kq_filter, EV_ADD, 0, 0, NULL);
    ret = kevent(event_set->epfd, &kqev, 1, NULL, 0, NULL);
    if (ret < 0) {
        ucs_error("kevent(epfd=%d, ADD, fd=%d) failed: %m", event_set->epfd,
                  event_fd);
        return UCS_ERR_IO_ERROR;
    }

#endif
    return UCS_OK;
}

ucs_status_t ucs_event_set_mod(ucs_sys_event_set_t *event_set, int event_fd,
                               ucs_event_set_type_t events)
{
#ifdef HAVE_SYS_EPOLL_H
    struct epoll_event raw_event;
#else
    struct kevent kqev;
    int kq_filter;
#endif
    int ret;

#ifdef HAVE_SYS_EPOLL_H
    memset(&raw_event, 0, sizeof(raw_event));
    raw_event.events   = ucs_event_set_map_to_raw_events(events);
    raw_event.data.fd  = event_fd;

    ret = epoll_ctl(event_set->epfd, EPOLL_CTL_MOD, event_fd, &raw_event);
    if (ret < 0) {
        ucs_error("epoll_ctl(epfd=%d, MOD, fd=%d) failed: %m", event_set->epfd,
                  event_fd);
        return UCS_ERR_IO_ERROR;
    }
#else
    memset(&kqev, 0, sizeof(kqev));
    kq_filter = ucs_event_set_map_to_raw_events(events);
#if 0
    EV_SET(&kqev, event_fd, kq_filter, EV_MOD, 0, 0, NULL);
#endif
    ret = kevent(event_set->epfd, &kqev, 1, NULL, 0, NULL);
    if (ret < 0) {
        ucs_error("kevent(epfd=%d, MOD, fd=%d) failed: %m", event_set->epfd,
                  event_fd);
        return UCS_ERR_IO_ERROR;
    }
#endif

    return UCS_OK;
}

ucs_status_t ucs_event_set_del(ucs_sys_event_set_t *event_set, int event_fd)
{
#ifndef HAVE_SYS_EPOLL_H
    struct kevent kqev;
#endif
    int ret;

#ifdef HAVE_SYS_EPOLL_H
    ret = epoll_ctl(event_set->epfd, EPOLL_CTL_DEL, event_fd, NULL);
    if (ret < 0) {
        ucs_error("epoll_ctl(epfd=%d, DEL, fd=%d) failed: %m", event_set->epfd,
                  event_fd);
        return UCS_ERR_IO_ERROR;
    }
#else
    /* TODO */
    EV_SET(&kqev,event_fd,EVFILT_READ, EV_DELETE, 0, 0, NULL);
    ret = kevent(event_set->epfd, &kqev, 1, NULL, 0, NULL);
    if (ret < 0) {
        ucs_error("kevent(epfd=%d, DEL, fd=%d) failed: %m", event_set->epfd,
                  event_fd);
        return UCS_ERR_IO_ERROR;
    }
#endif

    return UCS_OK;
}

ucs_status_t ucs_event_set_wait(ucs_sys_event_set_t *event_set, int timeout_ms,
                                ucs_event_set_handler_t event_set_handler,
                                void *arg)
{
    int nready;
#ifdef HAVE_SYS_EPOLL_H
    struct epoll_event ep_events[UCS_EVENT_EPOLL_MAX_EVENTS];
#else
    struct kevent kqev[UCS_EVENT_EPOLL_MAX_EVENTS];
    struct timespec kq_tmo;
#endif
    int i;

    if (event_set_handler == NULL) {
        return UCS_ERR_INVALID_PARAM;
    }

#ifdef HAVE_SYS_EPOLL_H
    nready = epoll_wait(event_set->epfd, ep_events, UCS_EVENT_EPOLL_MAX_EVENTS,
                        timeout_ms);
    if ((nready < 0) && (errno != EINTR)) {
        ucs_error("epoll_wait() failed: %m");
        return UCS_ERR_IO_ERROR;
    }
    ucs_trace_data("epoll_wait(epfd=%d, timeout=%d) returned %d",
                   event_set->epfd, timeout_ms, nready);
#else
    kq_tmo.tv_sec = timeout_ms / 1000;
    kq_tmo.tv_nsec = timeout_ms % 1000 * 1000;
    nready = kevent(event_set->epfd, NULL, 0, kqev, UCS_EVENT_EPOLL_MAX_EVENTS,
                    &kq_tmo);

    /* TODO */
    if (nready < 0) {
        ucs_error("kevent() failed: %m");
        return UCS_ERR_IO_ERROR;
    }
    ucs_trace_data("kevent(epfd=%d, timeout=%d) returned %d",
                   event_set->epfd, timeout_ms, nready);
#endif

    for (i=0; i < nready; i++) {
        int events;
#ifdef HAVE_SYS_EPOLL_H
        events = ucs_event_set_map_to_events(ep_events[i].events);
        event_set_handler(ep_events[i].data.fd, events, arg);
#else
        events = ucs_event_set_map_to_events(kqev[i].filter);
        event_set_handler(kqev[i].ident, events, arg);
#endif
    }
    return UCS_OK;
}

void ucs_event_set_cleanup(ucs_sys_event_set_t *event_set)
{
    close(event_set->epfd);
    ucs_free(event_set);
}
