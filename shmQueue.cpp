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

#define errExit(msg)    \
  do {                  \
    perror(msg);        \
    exit(EXIT_FAILURE); \
  } while (0)

static int futex(int *uaddr, int futex_op, int val,
                 const struct timespec *timeout, int *uaddr2, int val3) {
  return syscall(SYS_futex, uaddr, futex_op, val, timeout, uaddr, val3);
}

static void fwait(int *futexp) {
  int s;

  while (1) {
    if (__sync_bool_compare_and_swap(futexp, 1, 0)) break;

    s = futex(futexp, FUTEX_WAIT, 0, NULL, NULL, 0);
    if (s == -1 && errno != EAGAIN) errExit("futex-FUTEX_WAIT");
  }
}

static void fpost(int *futexp) {
  int s;
  if (__sync_bool_compare_and_swap(futexp, 0, 1)) {
    s = futex(futexp, FUTEX_WAKE, 1, NULL, NULL, 0);
    if (s == -1) errExit("futex-FUTEX_WAKE");
  }
}

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

bool fd_wait_read(int fd, struct timespec * ts)
{
  struct timespec start, end, intval, realint;
  struct timeval tv, *ptv = &tv;
  clock_gettime(CLOCK_MONOTONIC, &start);

  while(true) {
    if (ts) {
      clock_gettime(CLOCK_MONOTONIC, &end);
      timespec_diff(&start, &end, &intval);
      timespec_diff(&intval, ts, &realint);
      if (realint.tv_sec < 0 || realint.tv_nsec < 0) {return false;}
      TIMESPEC_TO_TIMEVAL(ptv, &realint);
    } else {
      ptv = NULL;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    int ret = select(fd+1, &read_fds, NULL, NULL, ptv);
    if((ret > 0) && FD_ISSET(fd, &read_fds))
    {
      return true;
    }
  }
}

struct CreateQueueResponse {
  size_t size;
};

SHMQueueServer::SHMQueueServer(const std::string &sock_path, size_t size): size_(size) {
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

SHMQueue* SHMQueueServer::accept(struct timespec *ts) {
  char tmp_path[256] = {0};
  fd_wait_read(sockfd_, ts);
  int client_fd = ::accept(sockfd_, NULL, NULL);
  if (client_fd < 0) {
    errExit("socket accept");
  }

  snprintf(tmp_path, sizeof(tmp_path), "cb.%d.%d", time(NULL), rand());

  int can_consume_fd = eventfd(0, EFD_CLOEXEC);
  int can_produce_fd = eventfd(0, EFD_CLOEXEC);
  int fd = shm_open(tmp_path, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
  ftruncate(fd, sizeof(SHMHeader) + size_);
  shm_unlink(tmp_path);
  SHMQueue *queue = new SHMQueue(fd, can_consume_fd, can_produce_fd, size_, true);

  CreateQueueResponse resp;
  resp.size = size_;
  iovec iov[1];
  iov[0].iov_base = &resp;
  iov[0].iov_len = sizeof(resp);

  int cmsgsize = CMSG_LEN(sizeof(int) * 3);
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

  ((int *)CMSG_DATA(cmptr))[0] = fd;
  ((int *)CMSG_DATA(cmptr))[1] = can_consume_fd;
  ((int *)CMSG_DATA(cmptr))[2] = can_produce_fd;

  if (sendmsg(client_fd, &msg, 0) < 0) {
    perror("sendmsg");
    delete queue;
    queue = NULL;
  }
  free(cmptr);
  close(client_fd);
  return queue;
}

#define ADDR(offset) ((char *)this->header_ + sizeof(SHMHeader) + (offset))
#define HEAD ADDR(0)

#define START ADDR(this->header_->start)
#define END ADDR(this->header_->end)

struct lk {
  lk(int *futexp) : f_(futexp) { fwait(f_); }
  ~lk() { fpost(f_); }

  private:
  int *f_;
};

SHMQueue::SHMQueue(int fd, int can_consume_fd, int can_produce_fd, size_t size, bool init) : fd_(fd), can_consume_fd_(can_consume_fd), can_produce_fd_(can_produce_fd) {
  size_t map_size = size + sizeof(SHMHeader);
  void *ptr = mmap(NULL, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (ptr == MAP_FAILED) {
    errExit("map failed");
  }
  header_ = static_cast<SHMHeader *>(ptr);
  if (init) {
    header_->length = size;
    header_->futexp = 1;
    header_->start = 0;
    header_->end = 0;
  }
  set_blocking(can_consume_fd, false);
  set_blocking(can_produce_fd, false);
}

SHMQueue* SHMQueue::connect(const std::string &sock_path) {
  ////////////////////////////
  struct sockaddr_un addr;
  int sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (sockfd < 0) {
    return NULL;
  }

  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, sock_path.c_str(), sizeof(addr.sun_path));

  if(::connect(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    errExit("socket connect");
  }

  int cmsgsize = CMSG_LEN(sizeof(int) * 3);
  cmsghdr *cmptr = (cmsghdr *)malloc(cmsgsize);

  CreateQueueResponse resp;
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

  int fd = ((int *)CMSG_DATA(cmptr))[0];
  int can_consume_fd = ((int *)CMSG_DATA(cmptr))[1];
  int can_produce_fd = ((int *)CMSG_DATA(cmptr))[2];
  free(cmptr);
  return new SHMQueue(fd, can_consume_fd, can_produce_fd, resp.size, false);
}

size_t SHMQueue::reserved() {
  if (header_->end >= header_->start) {
    return header_->length - header_->end + header_->start;
  } else {
    return header_->start - header_->end;
  }
}

int SHMQueue::push(const char *memory, size_t length) {
  if (header_->end >= header_->start) {
    size_t tail_len = header_->length - header_->end;
    if (tail_len > length) {
      memcpy(END, memory, length);
      header_->end += length;
    } else {
      size_t start_len = length - tail_len;
      memcpy(END, memory, tail_len);
      memcpy(HEAD, memory + tail_len, start_len);
      header_->end = start_len;
    }
  } else {
    memcpy(END, memory, length);
    header_->end += length;
  }
  return 0;
}

int SHMQueue::enqueue(const char *memory, size_t length, struct timespec *ts) {
  lk _(&header_->futexp);
  while (length + sizeof(size_t) >= reserved()) {
      int64_t cnt = 0;
      fpost(&header_->futexp);
      bool ret = fd_wait_read(can_produce_fd_, ts);
      fwait(&header_->futexp);
      if (!ret) return -1;
      read(can_produce_fd_, &cnt, sizeof(cnt));
  }
  push((char *)&length, sizeof(size_t));
  push(memory, length);
  int64_t cnt = 1;
  write(can_consume_fd_, &cnt, sizeof(cnt));
  return 0;
}

int SHMQueue::pop(char *memory, size_t length) {
  if (header_->end < header_->start) {
    size_t head_len = header_->length - header_->start;
    if (head_len >= length) {
      memcpy(memory, START, length);
      header_->start += length;
    } else {
      size_t start_len = length - head_len;
      memcpy(memory, START, head_len);
      memcpy(memory + head_len, HEAD, start_len);
      header_->start = start_len;
    }
  } else {
    memcpy(memory, START, length);
    header_->start += length;
  }
  return 0;
}

char *SHMQueue::dequeue(size_t *length, struct timespec *ts) {
  lk _(&header_->futexp);
  while ((header_->start - header_->end + header_->length) % header_->length == 0) {
      int64_t cnt = 0;
      fpost(&header_->futexp);
      bool ret = fd_wait_read(can_consume_fd_, ts);
      fwait(&header_->futexp);
      if (!ret) return NULL;
      int rc = read(can_consume_fd_, &cnt, sizeof(cnt));
  }

  pop((char *)(length), sizeof(size_t));
  char *ptr = (char *)malloc(*length);
  pop(ptr, *length);
  int64_t cnt = 1;
  write(can_produce_fd_, &cnt, sizeof(cnt));
  return ptr;
}

size_t SHMQueue::used() {
  lk _(&header_->futexp);
  return header_->length - reserved();
}

int main() {
  int count = 200000;
  int idx = 0;

  uint64_t digest = 5381L;
  if (fork()) {  // parent
    //ProfilerStart("perf_parent.out");
    SHMQueueServer server("/tmp/eventfd_test.sock", 2000);
    SHMQueue *queue = server.accept();
    const int max_size = 1024;
    char buf[max_size];
    FILE *f = fopen("/dev/urandom", "r");
    while (true) {
      idx++;
      size_t size = random() % max_size;
      fread(buf, size, 1, f);
      if (queue->enqueue(buf, size) == 0) {
        for (int i = 0; i < size; ++i) { digest = ((digest << 5) + digest) + buf[i]; }
        if (!--count) break;
      }
    }
    fclose(f);

    printf("parent: %lx idx: %d\n", digest, idx);

    //ProfilerStop();

    int status;
    wait(&status);
  } else {
    usleep(100000);
    //ProfilerStart("perf_child.out");
    SHMQueue *queue = SHMQueue::connect("/tmp/eventfd_test.sock");
    size_t length;
    while (true) {
      idx++;
      char *ptr = queue->dequeue(&length);
      if (ptr) {
        for (int i = 0; i < length; ++i) { digest = ((digest << 5) + digest) + ptr[i]; }
        free(ptr);
        if (!--count) break;
      }
    }
    assert(!queue->used());
    printf("child : %lx idx: %d\n", digest, idx);
    //ProfilerStop();
  }
  return 0;
}