/**
 * Copyright (C) Mellanox Technologies Ltd. 2001-2016.  ALL RIGHTS RESERVED.
 *
 * See file LICENSE for terms.
 */

#include "tcp.h"

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include <ucs/sys/string.h>
#ifdef __APPLE__
#include <ifaddrs.h>
#include <net/if_dl.h>
#include <sys/sysctl.h>
#include <sys/socket.h>
#include <net/route.h>
#include <netinet/in.h>
#include <net/if.h>
#else
#include <linux/sockios.h>
#include <linux/types.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#endif
#include <sys/ioctl.h>
#include <net/if_arp.h>
#include <net/if.h>
#include <netdb.h>


typedef ssize_t (*uct_tcp_io_func_t)(int fd, void *data, size_t size, int flags);

#ifdef __APPLE__
#define SPEED_MBPS 1000000
#define ETH_HLEN        14              /* Total octets in header.       */
#define ETH_FCS_LEN     4               /* Octets in the FCS             */
/* FIXME: change function name */
static ucs_status_t link_speed(const char *if_name, struct if_data *data) {
    struct ifaddrs *ifap, *ifaptr;
    uint32_t speed;

    if (getifaddrs(&ifap) == 0) {
        for(ifaptr = ifap; ifaptr != NULL; ifaptr = (ifaptr)->ifa_next) {
            if ((((ifaptr)->ifa_addr)->sa_family == AF_LINK) &&
                !strncmp((ifaptr)->ifa_name, if_name ,strlen(if_name))) {
                data = (struct if_data *)ifaptr->ifa_data;
                speed = data->ifi_baudrate; /* bps */
                break;
            }
        }
        freeifaddrs(ifap);
        return UCS_OK;
    } else {
        return UCS_ERR_IO_ERROR;
    }
}
#endif

ucs_status_t uct_tcp_netif_caps(const char *if_name, double *latency_p,
                                double *bandwidth_p)
{
    size_t mtu;
    ucs_status_t status;
    uint32_t speed_mbps;
    size_t ll_headers;
#ifdef __APPLE__
    struct if_data data;
    uint32_t speed_bps;
#else
    struct ethtool_cmd edata;
    struct ifreq ifr;
    int speed_known;
    short ether_type;
#endif

#ifdef __APPLE__
    status = link_speed(if_name, &data);
    if (status == UCS_OK) {
        mtu = data.ifi_baudrate;
        speed_bps = data.ifi_baudrate;
        speed_mbps = speed_bps / SPEED_MBPS;
        ucs_debug("speed of %s is %d Mbps. MTU: %zu", if_name, speed_mbps, mtu);
    } else {
        mtu = 1500;
        speed_mbps = 100;
        ucs_debug("speed of %s is UNKNOWN, assuming %d Mbps", if_name, speed_mbps);
    }

    /* FIXME: macOS supports ETHERNET only. */
    /* https://en.wikipedia.org/wiki/Ethernet_frame */
    ll_headers = 7 + /* preamble */
                 1 + /* start-of-frame */
                 ETH_HLEN + /* src MAC + dst MAC + ethertype */
                 ETH_FCS_LEN + /* CRC */
                 12; /* inter-packet gap */
#else
    memset(&ifr, 0, sizeof(ifr));

    speed_known  = 0;

    edata.cmd    = ETHTOOL_GSET;
    ifr.ifr_data = (void*)&edata;
    status = ucs_netif_ioctl(if_name, SIOCETHTOOL, &ifr);
    if (status == UCS_OK) {
#if HAVE_DECL_ETHTOOL_CMD_SPEED
        speed_mbps = ethtool_cmd_speed(&edata);
#else
        speed_mbps = edata.speed;
#endif
#if HAVE_DECL_SPEED_UNKNOWN
        speed_known = speed_mbps != (uint32_t)SPEED_UNKNOWN;
#else
        speed_known = (speed_mbps != 0) && ((uint16_t)speed_mbps != (uint16_t)-1);
#endif
    }

    if (!speed_known) {
        speed_mbps = 100;
        ucs_debug("speed of %s is UNKNOWN, assuming %d Mbps", if_name, speed_mbps);
    }

    status = ucs_netif_ioctl(if_name, SIOCGIFHWADDR, &ifr);
    if (status == UCS_OK) {
        ether_type = ifr.ifr_addr.sa_family;
    } else {
        ether_type = ARPHRD_ETHER;
    }

    status = ucs_netif_ioctl(if_name, SIOCGIFMTU, &ifr);
    if (status == UCS_OK) {
        mtu = ifr.ifr_mtu;
    } else {
        mtu = 1500;
    }

    switch (ether_type) {
    case ARPHRD_ETHER:
        /* https://en.wikipedia.org/wiki/Ethernet_frame */
        ll_headers = 7 + /* preamble */
                     1 + /* start-of-frame */
                     ETH_HLEN + /* src MAC + dst MAC + ethertype */
                     ETH_FCS_LEN + /* CRC */
                     12; /* inter-packet gap */
        break;
    case ARPHRD_INFINIBAND:
        ll_headers = /* LRH */   8  +
                     /* GRH */   40 +
                     /* BTH */   12 +
                     /* DETH */  8  +
                     /* IPoIB */ 4 + 20 +
                     /* ICRC */  4  +
                     /* VCRC */  2  +
                     /* DELIM */ 2;
        break;
    default:
        ll_headers = 0;
        break;
    }
#endif

    /* https://w3.siemens.com/mcms/industrial-communication/en/rugged-communication/Documents/AN8.pdf */
    *latency_p   = 576.0 / (speed_mbps * 1e6) + 5.2e-6;
    *bandwidth_p = (speed_mbps * 1e6) / 8 *
                   (mtu - 40) / (mtu + ll_headers); /* TCP/IP header is 40 bytes */
    return UCS_OK;
}

ucs_status_t uct_tcp_netif_inaddr(const char *if_name, struct sockaddr_in *ifaddr,
                                  struct sockaddr_in *netmask)
{
    ucs_status_t status;
    struct ifreq ifra, ifrnm;

    status = ucs_netif_ioctl(if_name, SIOCGIFADDR, &ifra);
    if (status != UCS_OK) {
        return status;
    }

    if (netmask != NULL) {
        status = ucs_netif_ioctl(if_name, SIOCGIFNETMASK, &ifrnm);
        if (status != UCS_OK) {
            return status;
        }
    }

    if ((ifra.ifr_addr.sa_family != AF_INET) ) {
        ucs_error("%s address is not INET", if_name);
        return UCS_ERR_INVALID_ADDR;
    }

    memcpy(ifaddr,  (struct sockaddr_in*)&ifra.ifr_addr,  sizeof(*ifaddr));
    if (netmask != NULL) {
        memcpy(netmask, (struct sockaddr_in*)&ifrnm.ifr_addr, sizeof(*netmask));
    }

    return UCS_OK;
}

static ucs_status_t uct_tcp_netif_cmp_get_default_iface(char *ifname)
{
    int mib[6];
    size_t needed;
    char *buf, *next, *lim;
    struct rt_msghdr2 *rtm;

    mib[0] = CTL_NET;
    mib[1] = PF_ROUTE;
    mib[2] = 0;
    mib[3] = 0;
    mib[4] = NET_RT_DUMP2;
    mib[5] = 0;

    if( sysctl(mib, 6, NULL, &needed, NULL,0) < 0){
        ucs_error("sysctl");
        return UCS_ERR_IO_ERROR;
    }

    if((buf = malloc(needed)) == 0 ){
        ucs_error("malloc");
        return UCS_ERR_IO_ERROR;
    }

    if( sysctl(mib, 6, buf, &needed, NULL,0) < 0){
        ucs_error("sysctl");
        return UCS_ERR_IO_ERROR;
    }

    lim  = buf + needed;
    for (next = buf; next < lim; next += rtm->rtm_msglen) {
        struct sockaddr *sa;
        rtm = (struct rt_msghdr2 *)next;

        sa = (struct sockaddr *)(rtm + 1);
        if(sa->sa_family == AF_INET) {
            off_t off_dest = sizeof(struct sockaddr)*(RTA_DST-1);
            off_t off_mask = sizeof(struct sockaddr)*(RTA_NETMASK);

            struct sockaddr_in *dest = (struct sockaddr_in *)sa + off_dest;
            struct sockaddr_in *mask = (struct sockaddr_in *)sa + off_mask;

            /* address: 0.0.0.0/0 */
            if( rtm->rtm_addrs & RTA_DST &&
                dest->sin_addr.s_addr == INADDR_ANY &&
                rtm->rtm_addrs & RTA_NETMASK &&
                ((ntohl(mask->sin_addr.s_addr) == 0L))) {

                if_indextoname(rtm->rtm_index, ifname);
                return UCS_OK;
            }
        }
    }
    return UCS_ERR_IO_ERROR;
}

ucs_status_t uct_tcp_netif_is_default(const char *if_name, int *result_p)
{
#ifdef __APPLE__
    char def_ifname[IFNAMSIZ + 1];
#else
    static const char *filename = "/proc/net/route";
    in_addr_t netmask;
    char name[128];
    char str[128];
    FILE *f;
#endif
    int ret;

#ifdef __APPLE__
    ret = uct_tcp_netif_cmp_get_default_iface(def_ifname);
    if (ret != UCS_OK) {
        return UCS_ERR_IO_ERROR;
    }
    if(strcmp(def_ifname,if_name) == 0) {
        *result_p = 1;
    } else {
        *result_p = 0;
    }
    return UCS_OK;
#else
    f = fopen(filename, "r");
    if (f == NULL) {
        ucs_debug("failed to open '%s': %m", filename);
        return UCS_ERR_IO_ERROR;
    }

    /*
    Iface  Destination  Gateway  Flags  RefCnt  Use  Metric  Mask  MTU  Window  IRTT
    */
    while (fgets(str, sizeof(str), f) != NULL) {
        ret = sscanf(str, "%s %*x %*x %*d %*d %*d %*d %x", name, &netmask);
        if ((ret == 3) && !strcmp(name, if_name) && (netmask == 0)) {
            *result_p = 1;
            break;
        }

        /* Skip rest of the line */
        while ((strchr(str, '\n') == NULL) && (fgets(str, sizeof(str), f) != NULL));
    }

    *result_p = 0;
    fclose(f);
    return UCS_OK;
#endif
}
