#include "surveillance.hpp"

V4L2Camera::V4L2Camera(SystemStatus* s, Logger* l, FrameBuffer* p, int ps) 
    : status(s), logger(l), pool(p), pool_size(ps) {}

V4L2Camera::~V4L2Camera() {
    stop();
}

bool V4L2Camera::initDevice() {
    fd = open(config::CAM_DEVICE, O_RDWR | O_NONBLOCK, 0);
    if (fd < 0) {
        logger->error(std::string("Cannot open device: ") + config::CAM_DEVICE);
        return false;
    }

    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        logger->error("VIDIOC_QUERYCAP failed");
        return false;
    }

    logger->info(std::string("Camera: ") + (char*)cap.card);

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = config::CAM_WIDTH;
    fmt.fmt.pix.height = config::CAM_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        logger->error("VIDIOC_S_FMT failed");
        return false;
    }

    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
        logger->warn("Camera does not support YUYV, trying MJPEG");
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            logger->error("Failed to set MJPEG format");
            return false;
        }
    }

    logger->info("Format set: " + std::to_string(fmt.fmt.pix.width) + "x" + 
                 std::to_string(fmt.fmt.pix.height));

    return true;
}

bool V4L2Camera::initBuffers() {
    memset(&req, 0, sizeof(req));
    req.count = config::BUFFER_POOL_SIZE;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        logger->error("VIDIOC_REQBUFS failed");
        return false;
    }

    buffers.resize(req.count);
    for (size_t i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            logger->error("VIDIOC_QUERYBUF failed");
            return false;
        }

        buffers[i].length = buf.length;
        buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd, buf.m.offset);

        if (MAP_FAILED == buffers[i].start) {
            logger->error("mmap failed");
            return false;
        }
    }

    for (size_t i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        ioctl(fd, VIDIOC_QBUF, &buf);
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        logger->error("VIDIOC_STREAMON failed");
        return false;
    }

    return true;
}

void V4L2Camera::convertYUYVtoBGR(const uint8_t* yuyv, cv::Mat& bgr, int width, int height) {
    bgr.create(height, width, CV_8UC3);
    cv::Mat yuyv_mat(height, width, CV_8UC2, const_cast<uint8_t*>(yuyv));
    cv::cvtColor(yuyv_mat, bgr, cv::COLOR_YUV2BGR_YUYV);
}

void V4L2Camera::captureLoop() {
    fd_set fds;
    struct timeval tv;
    int r;

    while (running) {
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        r = select(fd + 1, &fds, NULL, NULL, &tv);
        if (r <= 0) continue;

        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) continue;
            logger->error("VIDIOC_DQBUF failed");
            break;
        }

        FrameBuffer* fb = nullptr;
        for (int i = 0; i < pool_size; i++) {
            if (!pool[i].in_use.load()) {
                fb = &pool[i];
                break;
            }
        }

        if (fb) {
            fb->in_use = true;
            fb->timestamp = std::chrono::steady_clock::now();
            fb->frame_number = status->total_frames.load();

            if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
                convertYUYVtoBGR((uint8_t*)buffers[buf.index].start, fb->frame, 
                                fmt.fmt.pix.width, fmt.fmt.pix.height);
            } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG) {
                std::vector<uint8_t> jpeg_data((uint8_t*)buffers[buf.index].start,
                                               (uint8_t*)buffers[buf.index].start + buf.bytesused);
                fb->frame = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
            }

            cv::Mat gray;
            cv::cvtColor(fb->frame, gray, cv::COLOR_BGR2GRAY);
            fb->brightness = cv::mean(gray)[0];

            status->total_frames++;
        }

        ioctl(fd, VIDIOC_QBUF, &buf);
    }
}

bool V4L2Camera::start() {
    if (!initDevice() || !initBuffers()) {
        return false;
    }

    running = true;
    capture_thread = std::thread(&V4L2Camera::captureLoop, this);
    logger->info("Camera capture started");
    return true;
}

void V4L2Camera::stop() {
    running = false;
    if (capture_thread.joinable()) capture_thread.join();

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);

    for (auto& buf : buffers) {
        if (buf.start != MAP_FAILED) {
            munmap(buf.start, buf.length);
        }
    }

    if (fd >= 0) close(fd);
    logger->info("Camera stopped");
}

FrameBuffer* V4L2Camera::acquireFrame() {
    for (int i = 0; i < pool_size; i++) {
        if (pool[i].in_use.load()) {
            return &pool[i];
        }
    }
    return nullptr;
}

void V4L2Camera::releaseFrame(FrameBuffer* fb) {
    if (fb) fb->in_use = false;
}
