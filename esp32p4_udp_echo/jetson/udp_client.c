/* Simple UDP echo client for NVIDIA Jetson (Linux / POSIX sockets).
 * Usage: ./udp_client 192.168.7.2 "Hello P4!"
 */
#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define UDP_PORT 3333
#define BUFFER_CAP 512

static double elapsed_ms(const struct timespec *start,
                         const struct timespec *end)
{
    return (double)(end->tv_sec - start->tv_sec) * 1000.0 +
           (double)(end->tv_nsec - start->tv_nsec) / 1000000.0;
}

int main(int argc, char **argv)
{
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <p4-ip> <message>\n", argv[0]);
        fprintf(stderr, "Example: %s 192.168.7.2 'Hello P4!'\n", argv[0]);
        return 1;
    }

    const char *message = argv[2];
    size_t message_len = strlen(message);
    if (message_len > BUFFER_CAP) {
        fprintf(stderr, "Message too long (max %d bytes)\n", BUFFER_CAP);
        return 1;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    struct timeval timeout = { .tv_sec = 2, .tv_usec = 0 };
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                   &timeout, sizeof(timeout)) < 0) {
        perror("setsockopt");
        close(sock);
        return 1;
    }

    struct sockaddr_in p4 = {0};
    p4.sin_family = AF_INET;
    p4.sin_port = htons(UDP_PORT);
    if (inet_pton(AF_INET, argv[1], &p4.sin_addr) != 1) {
        fprintf(stderr, "Invalid P4 IP address: %s\n", argv[1]);
        close(sock);
        return 1;
    }

    struct timespec before, after;
    clock_gettime(CLOCK_MONOTONIC, &before);
    ssize_t sent = sendto(sock, message, message_len, 0,
                          (struct sockaddr *)&p4, sizeof(p4));
    if (sent != (ssize_t)message_len) {
        perror("sendto");
        close(sock);
        return 1;
    }

    printf("Sent %zd bytes to %s:%d: %s\n", sent, argv[1], UDP_PORT, message);
    char received[BUFFER_CAP + 1];
    struct sockaddr_in source = {0};
    socklen_t source_len = sizeof(source);
    ssize_t n = recvfrom(sock, received, BUFFER_CAP, 0,
                         (struct sockaddr *)&source, &source_len);
    clock_gettime(CLOCK_MONOTONIC, &after);
    if (n < 0) {
        perror("recvfrom (no echo received within 2 seconds)");
        close(sock);
        return 1;
    }
    received[n] = '\0';
    printf("Received %zd bytes: %.*s\n", n, (int)n, received);
    printf("Round-trip time: %.3f ms\n", elapsed_ms(&before, &after));

    // Verify bytes and source, not just whether any UDP datagram arrived.
    if (source.sin_addr.s_addr != p4.sin_addr.s_addr ||
        source.sin_port != p4.sin_port ||
        (size_t)n != message_len || memcmp(received, message, message_len) != 0) {
        fprintf(stderr, "FAIL: response sender or payload did not match\n");
        close(sock);
        return 1;
    }
    printf("PASS: UDP echo matched exactly!\n");
    close(sock);
    return 0;
}
