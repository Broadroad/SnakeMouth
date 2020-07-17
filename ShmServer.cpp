#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/futex.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/wait.h>
//#include <gperftools/profiler.h>
#include <sys/select.h>
#include "ShmServer.h"

#define errExit(msg)    \
  do {                  \
    perror(msg);        \
    exit(EXIT_FAILURE); \
  } while (0)

ShmQueue::ShmQueue(int fd, int can_consume_fd, int can_produce_fd, size_t size) {
    
}
struct CreateQueueResponse {
  size_t size;
};

bool fd_wait_read(int fd, struct timespec *ts) {
    struct timespec start, end, intval, realint;
    struct timeval tv, *ptv = &tv;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (true) {
        if (ts) {
            clock_gettime(CLOCK_MONOTONIC, &end);
            timespec_diff(&start, &end, &intval);
            timespec_diff(&intval, ts, &realint);
            if (realint.tv_sec < 0 || realint.tv_nsec < 0) {
                return false;
            }
            TIMESPEC_TO_TIMEVAL(ptv, &realint);
        } else {
            ptv = NULL;
        }

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);

        int ret = select(fd + 1, &read_fds, NULL, NULL, ptv);
        if ((ret > 0) && FD_ISSET(fd, &read_fds)) {
            return true;
        }
    }
}

ShmServer::ShmServer(const std::string &sock_path, size_t size) : size_(size) {
    struct sockaddr_un addr;
    if ((sockfd_ = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
        errExit("create socket");
        return;
    }
    set_blocking(sockfd_, false);
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, sock_path.c_str(), sizeof(addr.sun_path));
    unlink(sock_path.c_str());
    if (bind(sockfd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        errExit("socket bind");
        return;
    }
    if (listen(sockfd_, 5) < 0) {
        errExit("socket listen");
        return;
    }
}

void ShmServer::Accept(struct timespec *ts) {
    fd_wait_read(sockfd_, ts);
    int client_fd = ::accept(sockfd_, NULL, NULL);
    if (client_fd < 0) {
        errExit("socket accept");
    }

    can_consume_fd_ = eventfd(0, EFD_CLOEXEC);
    can_produce_fd_ = eventfd(0, EFD_CLOEXEC);
    CreateQueueResponse resp;
    resp.size = size_;
    iovec iov[1];
    iov[0].iov_base = &resp;
    iov[0].iov_len = sizeof(resp);
    
    int cmsgsize = CMSG_LEN(sizeof(int) * 2);
    cmsghdr *cmptr = (cmsghdr *)(malloc(cmsgsize));

    cmptr->cmsg_level = SOL_SOCKET;
    cmptr->cmsg_type = SCM_RIGHTS;
    cmptr->cmsg_len = cmsgsize;

    msghdr msg;
    msg.msg_iov = iov;
    msg.msg_iovlen = 1;
    msg.msg_name = NULL;
    msg.msg_namelen = 0;
    msg.msg_control = cmptr;
    msg.msg_controllen = cmsgsize;
    
    ((int *)CMSG_DATA(cmptr))[0] = can_consume_fd_;
    ((int *)CMSG_DATA(cmptr))[1] = can_produce_fd_;

    if (sendmsg(client_fd, &msg, 0) < 0) {
        perror("sendmsg");
    }
    free(cmptr);
    close(client_fd);
}

void Shm::Start() {
    for (int i = 0; i < size_; i++) {
        int64_t cnt = i;
        write(can_consume_fd_, &cnt, sizeof(cnt));
    }
    usleep(100000);
}

