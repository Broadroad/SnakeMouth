//
//#pragma once
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stddef.h>
#include <string>
#include <sys/time.h>
#include <time.h>

class SHMQueue;

struct SHMHeader {
  size_t length;
  int futexp;
  size_t start;
  size_t end;
};

class SHMQueueServer {
  public:
    SHMQueueServer(const std::string &sock_path, size_t size);
    SHMQueue* accept(struct timespec *ts=NULL);
    int fd() {return sockfd_;}
  private:
    int sockfd_;
    size_t size_;
};

class SHMQueue {
 public:
  SHMQueue(int fd, int can_consume_fd, int can_produce_fd, size_t size, bool init);
  static SHMQueue* connect(const std::string &sock_path);
  char *dequeue(size_t *length, struct timespec *ts=NULL);
  int enqueue(const char *memory, size_t length, struct timespec *ts=NULL);
  size_t used();

 private:
  int pop(char *memory, size_t length);
  int push(const char *memory, size_t length);
  size_t reserved();

 private:
  SHMHeader *header_;
  int fd_;
  int can_consume_fd_;
  int can_produce_fd_;
};

