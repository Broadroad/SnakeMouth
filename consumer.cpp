#include "consumer.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/futex.h>
#include <sys/eventfd.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/wait.h>
//#include <gperftools/profiler.h>
#include <sys/select.h>

using namespace std;
#define errExit(msg)    \
		do {                  \
				perror(msg);        \
				exit(EXIT_FAILURE); \
		} while (0)
struct CreateQueueResponse {
  size_t size;
};

ShmClient::ShmClient(const std::string &sock_path, size_t size) : size_(size){
    struct sockaddr_un addr;
    int sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sockfd < 0) {
        return ;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, sock_path.c_str(), sizeof(addr.sun_path));

    if(::connect(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        errExit("socket connect");
    }

    int cmsgsize = CMSG_LEN(sizeof(int) * 2);
    cmsghdr *cmptr = (cmsghdr *)malloc(cmsgsize);
    
    CreateQueueResponse resp;
    resp.size = size_;
    iovec iov[1];
    iov[0].iov_base = &resp;
    iov[0].iov_len = sizeof(resp);

    msghdr msg;
    msg.msg_iov = iov;
    msg.msg_iovlen = 1;
    msg.msg_name = NULL;
    msg.msg_namelen = 0;
    msg.msg_control = cmptr;
    msg.msg_controllen = cmsgsize;

    if(recvmsg(sockfd, &msg, 0) < 0) {
        errExit("recvmsg");
    }

    can_consume_fd_ = ((int *)CMSG_DATA(cmptr))[0];
    can_produce_fd_ = ((int *)CMSG_DATA(cmptr))[1];
    free(cmptr);
}

void ShmClient::Connect() {
	printf("Connect, can_consumer_fd is %d\n", can_consume_fd_);
    int epollfd = epoll_create(1024);
    if (epollfd == -1) abort();
    struct epoll_event evnt = {0};
    evnt.data.fd = can_consume_fd_;
    evnt.events = EPOLLIN | EPOLLET;
    if (epoll_ctl(epollfd, EPOLL_CTL_ADD,can_consume_fd_, &evnt) == -1)
        abort();
    static const int EVENTS = 20;
    struct epoll_event evnts[EVENTS];
    while (1)
    {
        int count = epoll_wait(epollfd, evnts, EVENTS, 5000);
        if (count == -1)
        {
            if (errno != EINTR)
            {
                perror("epoll_wait");
                return;
            }
        }

        int i;
        for (i = 0; i < count; ++i)
        {
            struct epoll_event *e = evnts + i;
            if (e->data.fd == can_consume_fd_)
            {
                eventfd_t val;
                eventfd_read(can_consume_fd_, &val);
                printf("DING: %lld\n", (long long)val);
            }
        }
    }
}

int main() {
	ShmClient client("/tmp/test.socket", 10);
	usleep(1000);
	client.Connect();
	return 0;
}
