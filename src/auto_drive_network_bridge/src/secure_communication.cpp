#include "auto_drive_network_bridge/secure_communication.hpp"
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

SecureCommunication::SecureCommunication() : ctx(nullptr), ssl(nullptr), socket(-1) {
    init_openssl();
}

SecureCommunication::~SecureCommunication() {
    if (ssl) {
        SSL_free(ssl);
    }
    if (ctx) {
        SSL_CTX_free(ctx);
    }
    if (socket != -1) {
        close(socket);
    }
    cleanup_openssl();
}

void SecureCommunication::init_openssl() {
    SSL_load_error_strings();
    OpenSSL_add_ssl_algorithms();
}

void SecureCommunication::cleanup_openssl() {
    EVP_cleanup();
}

SSL_CTX* SecureCommunication::create_context() {
    const SSL_METHOD* method = TLS_method();
    SSL_CTX* ctx = SSL_CTX_new(method);
    if (!ctx) {
        std::cerr << "Unable to create SSL context" << std::endl;
        ERR_print_errors_fp(stderr);
        exit(EXIT_FAILURE);
    }
    return ctx;
}

void SecureCommunication::configure_context(SSL_CTX* ctx) {
    SSL_CTX_set_ecdh_auto(ctx, 1);

    if (SSL_CTX_use_certificate_file(ctx, "cert.pem", SSL_FILETYPE_PEM) <= 0) {
        ERR_print_errors_fp(stderr);
        exit(EXIT_FAILURE);
    }

    if (SSL_CTX_use_PrivateKey_file(ctx, "key.pem", SSL_FILETYPE_PEM) <= 0 ) {
        ERR_print_errors_fp(stderr);
        exit(EXIT_FAILURE);
    }
}

bool SecureCommunication::initialize(bool is_server) {
    ctx = create_context();
    configure_context(ctx);

    if (is_server) {
        // Server specific initialization
        ssl = SSL_new(ctx);
        if (!ssl) {
            std::cerr << "Error creating SSL" << std::endl;
            return false;
        }
    } else {
        // Client specific initialization
        // Nothing specific for client in this case
    }

    return true;
}

bool SecureCommunication::connect(const std::string& host, int port) {
    socket = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return false;
    }

    sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address" << std::endl;
        return false;
    }

    if (::connect(socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Connection failed" << std::endl;
        return false;
    }

    ssl = SSL_new(ctx);
    SSL_set_fd(ssl, socket);
    if (SSL_connect(ssl) != 1) {
        std::cerr << "SSL connection failed" << std::endl;
        return false;
    }

    return true;
}

bool SecureCommunication::accept(int port) {
    int server_socket = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return false;
    }

    sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        return false;
    }

    if (listen(server_socket, 1) < 0) {
        std::cerr << "Listen failed" << std::endl;
        return false;
    }

    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    socket = ::accept(server_socket, (struct sockaddr*)&client_addr, &client_len);
    if (socket < 0) {
        std::cerr << "Accept failed" << std::endl;
        return false;
    }

    SSL_set_fd(ssl, socket);
    if (SSL_accept(ssl) <= 0) {
        std::cerr << "SSL accept failed" << std::endl;
        return false;
    }

    close(server_socket);
    return true;
}

bool SecureCommunication::send(const std::string& message) {
    if (SSL_write(ssl, message.c_str(), message.length()) <= 0) {
        std::cerr << "SSL write failed" << std::endl;
        return false;
    }
    return true;
}

std::string SecureCommunication::receive() {
    char buffer[4096];
    int len = SSL_read(ssl, buffer, sizeof(buffer));
    if (len <= 0) {
        std::cerr << "SSL read failed" << std::endl;
        return "";
    }
    return std::string(buffer, len);
}