#include "epoll_event_loop.hpp"

EpollEventLoop::EpollEventLoop() {
    epollfd = epoll_create1(0);
}

EpollEventLoop::~EpollEventLoop() {
    close(epollfd);
}

bool EpollEventLoop::register_event(EvtId* p_evt, int fd, uint32_t events, const Callback& callback) {
    EventContext* ctx = new EventContext{ fd, callback };
    struct epoll_event ev = {
        .events = events,
        .data = { .ptr = ctx }
    };
    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &ev) == -1) {
        delete ctx; // Cleanup the dynamically allocated EventContext in case of failure
        return false;
    }

    if (p_evt) *p_evt = ctx;

    n_events_++;
    return true;
}

bool EpollEventLoop::deregister_event(EvtId evt) {
    if (evt == nullptr) return false;
    if (epoll_ctl(epollfd, EPOLL_CTL_DEL, evt->fd, nullptr) == -1) return false;
    drop_event(evt);
    delete evt;
    return true;
}

bool EpollEventLoop::run_until_empty() {
    while (n_events_) {
        n_triggered_events_ = epoll_wait(epollfd, triggered_events_, kMaxEventsPerIteration, -1);
        if (n_triggered_events_ == -1) return false;
        for (int i = 0; i < n_triggered_events_; ++i) {
            EventContext* handler = static_cast<EventContext*>(triggered_events_[i].data.ptr);
            handler->callback(triggered_events_[i].events);
        }
    }
    return true;
}

void EpollEventLoop::drop_event(EvtId evt) {
    for (int i = 0; i < n_triggered_events_; ++i) {
        if (reinterpret_cast<EventContext*>(triggered_events_[i].data.ptr) == evt) {
            triggered_events_[i].data.ptr = nullptr;
        }
    }
}

bool EpollEvent::init(EpollEventLoop* event_loop, const Callback& callback) {
    event_loop_ = event_loop;
    callback_ = callback;

    fd_ = eventfd(0, 0);
    if (fd_ < 0) return false;

    if (!event_loop->register_event(&evt_, fd_, EPOLLIN, std::bind(&EpollEvent::on_trigger, this, _1))) {
        close(fd_);
        std::cerr << "Failed to register event" << std::endl;
        return false;
    }

    return true;
}

void EpollEvent::deinit() {
    event_loop_->deregister_event(evt_);
    close(fd_);
    fd_ = -1;
}

bool EpollEvent::set() {
    const uint64_t val = 1;
    if (write(fd_, &val, sizeof(val)) != sizeof(val)) return false;
    return true;
}

void EpollEvent::on_trigger(uint32_t event_id) {
    uint64_t val;
    if (read(fd_, &val, sizeof(val)) != sizeof(val)) {
        std::cerr << "Failed to read eventfd" << std::endl;
        return;
    }

    callback_(event_id);
}
