#include "surveillance.hpp"

StreamServer::StreamServer(int p, SystemStatus* s, Logger* l) 
    : port(p), status(s), logger(l) {}

StreamServer::~StreamServer() {
    stop();
}

bool StreamServer::start() {
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        logger->error("Socket creation failed");
        return false;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        logger->error("Bind failed");
        return false;
    }

    if (listen(server_fd, config::MAX_CLIENTS) < 0) {
        logger->error("Listen failed");
        return false;
    }

    running = true;
    server_thread = std::thread(&StreamServer::serverLoop, this);
    logger->info("Stream server started on port " + std::to_string(port));
    return true;
}

void StreamServer::stop() {
    running = false;
    if (server_fd >= 0) close(server_fd);
    if (server_thread.joinable()) server_thread.join();

    for (auto& t : client_threads) {
        if (t.joinable()) t.join();
    }
}

void StreamServer::serverLoop() {
    while (running) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &addr_len);
        if (client_fd < 0) continue;

        logger->info("Client connected: " + std::string(inet_ntoa(client_addr.sin_addr)));

        client_threads.emplace_back(&StreamServer::handleClient, this, client_fd);
    }
}

void StreamServer::handleClient(int client_fd) {
    const char* headers = 
        "HTTP/1.1 200 OK
"
        "Content-Type: multipart/x-mixed-replace; boundary=--frame
"
        "Connection: keep-alive
"
        "Cache-Control: no-cache
"
        "
";

    send(client_fd, headers, strlen(headers), MSG_NOSIGNAL);

    while (running) {
        FrameBuffer* fb = nullptr;
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            fb = current_frame;
        }

        if (!fb || fb->frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            continue;
        }

        if (!sendMJPEG(client_fd, fb->frame)) {
            break;
        }
    }

    close(client_fd);
    logger->info("Client disconnected");
}

bool StreamServer::sendMJPEG(int client_fd, const cv::Mat& frame) {
    std::vector<uchar> jpeg_buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, config::STREAM_QUALITY};
    cv::imencode(".jpg", frame, jpeg_buffer, params);

    std::stringstream header;
    header << "--frame
"
           << "Content-Type: image/jpeg
"
           << "Content-Length: " << jpeg_buffer.size() << "
"
           << "
";

    std::string header_str = header.str();

    if (send(client_fd, header_str.c_str(), header_str.length(), MSG_NOSIGNAL) < 0)
        return false;

    if (send(client_fd, jpeg_buffer.data(), jpeg_buffer.size(), MSG_NOSIGNAL) < 0)
        return false;

    if (send(client_fd, "
", 2, MSG_NOSIGNAL) < 0)
        return false;

    return true;
}

void StreamServer::updateFrame(FrameBuffer* fb) {
    std::lock_guard<std::mutex> lock(frame_mutex);
    current_frame = fb;
}
