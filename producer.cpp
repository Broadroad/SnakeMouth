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
#include "producer.h"
#include <sys/time.h>
#include <time.h>


#define errExit(msg)    \
  do {                  \
    perror(msg);        \
    exit(EXIT_FAILURE); \
  } while (0)

struct CreateQueueResponse {
  size_t size;
};

bool set_blocking(int fd, bool blocking)
{
		if (fd < 0) return false;
		int flags = fcntl(fd, F_GETFL, 0);
		if (flags == -1) return false;
		flags = blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK);
		return (fcntl(fd, F_SETFL, flags) == 0) ? true : false;
}

void timespec_diff(struct timespec *start, struct timespec *stop,
                   struct timespec *result)
{
	if ((stop->tv_nsec - start->tv_nsec) < 0) {
		result->tv_sec = stop->tv_sec - start->tv_sec - 1;
		result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
	} else {
		result->tv_sec = stop->tv_sec - start->tv_sec;
		result->tv_nsec = stop->tv_nsec - start->tv_nsec;
	}

	return;
}
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
	printf("send msg success, can_consumer_fd_ is %d\n", can_consume_fd_);
    free(cmptr);
    close(client_fd);
}

void ShmServer::Start() {
    for (int i = 1; i < size_; i++) {
        int64_t cnt = i;
		printf("write %d\n", cnt);
        write(can_consume_fd_, &cnt, sizeof(cnt));
		usleep(10000);
    }
}

int main() {
	ShmServer server("/tmp/test.socket", 10);
	server.Accept();
	server.Start();
}
