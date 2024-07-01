#pragma once

#include <openssl/ssl.h>
#include <openssl/err.h>
#include <string>

class SecureCommunication {
public:
    SecureCommunication();
    ~SecureCommunication();

    bool initialize(bool is_server);
    bool connect(const std::string& host, int port);
    bool accept(int port);
    bool send(const std::string& message);
    std::string receive();

private:
    SSL_CTX* ctx;
    SSL* ssl;
    int socket;

    void init_openssl();
    void cleanup_openssl();
    SSL_CTX* create_context();
    void configure_context(SSL_CTX* ctx);
};